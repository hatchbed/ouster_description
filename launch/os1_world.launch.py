# Copyright (c) 2018, ouster-lidar
# Copyright (c) 2019, Wil Selby
# Copyright (c) 2020, Clearpath Robotics
# Copyright (c) 2024, Marc Alban / Hatchbed LLC
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.events.process import ProcessStarted
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


launch_logger = get_logger('gazebo_cleanup')

# Create a global list to store the exact PIDs assigned by the OS
gz_pids = []


def record_pids(event: ProcessStarted, context):
    """Capture the OS PID of the Gazebo wrapper the moment it starts."""
    global gz_pids
    # The launch system names the process 'gazebo-1', 'ruby', or 'gz'
    if 'gazebo' in event.process_name or 'gz' in event.process_name:
        gz_pids.append(event.pid)
        launch_logger.info(f'Recorded gazebo PID: {event.pid}')


def get_descendant_pids(pid):
    """Recursively find all descendant PIDs of a given parent PID."""
    pids = []
    try:
        # pgrep -P returns the direct children of the given PID
        output = subprocess.check_output(['pgrep', '-P', str(pid)])
        for child_pid in output.decode().split():
            pids.append(child_pid)
            # Recursively find children of this child (grandchildren, etc.)
            pids.extend(get_descendant_pids(child_pid))
    except subprocess.CalledProcessError:
        pass  # pgrep returns an error code if no children are found
    return pids


def kill_gazebo_tree(context, *args, **kwargs):
    """Surgically kill the recorded PIDs and their children on shutdown."""
    global gz_pids
    if len(gz_pids) == 0:
        return

    launch_logger.info('Cleaning up gazebo ...')
    for pid in gz_pids:
        descendants = get_descendant_pids(pid)

        if descendants:
            launch_logger.info(f'Terminating descendants of gazebo [{pid}] ...')
            # Reverse the list to kill grandchildren first (bottom-up execution)
            for child_pid in reversed(descendants):
                # Use SIGKILL (-9) on descendants to guarantee the server dies instantly
                launch_logger.info(f'Killing gazebo descendant [{child_pid}] ...')
                subprocess.run(['kill', '-9', str(child_pid)], stderr=subprocess.DEVNULL)

    gz_pids = []


def generate_launch_description():
    # Package Directories
    pkg_ouster_description = FindPackageShare('ouster_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    model = LaunchConfiguration('model')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')

    # Create a SetEnvironmentVariable action
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            ':',
            PathJoinSubstitution([pkg_ouster_description, '..']),
        ]
    )

    # 1. Declare Launch Arguments
    declare_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=PathJoinSubstitution([pkg_ouster_description, 'worlds', 'example.world'])
    )
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution(
            [pkg_ouster_description, 'urdf', 'example.urdf.xacro']
        )
    )
    declare_rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    declare_gui_arg = DeclareLaunchArgument('gui', default_value='true')

    # 2. Start Gazebo Sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [
                world_name,
                ' -r',
                PythonExpression(["' -s' if '", gui, "' == 'false' else ''"])],
        }.items()
    )

    # 3. Process Xacro file
    robot_description_content = Command(['xacro ', model])

    # 4. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    # 5. Spawn the robot into Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'example',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # 6. Bridge LiDAR and IMU topics from Gazebo to ROS 2
    # Note: These topics must match the <topic> tags in your xacro
    bridge_config = PathJoinSubstitution([pkg_ouster_description, 'config', 'bridge.yaml'])
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 7. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_ouster_description, 'rviz', 'example.rviz'])],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Captures PIDs of starting processes
    capture_pid_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            on_start=record_pids
        )
    )

    # Gazebo cleanup
    cleanup_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=kill_gazebo_tree)]
        )
    )

    # Shutdown Trigger
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bridge_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    ld = LaunchDescription([
        declare_use_sim_time_arg,
        declare_world_name_arg,
        declare_model_arg,
        declare_rviz_arg,
        declare_gui_arg,
        set_gz_resource_path,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        bridge_node,
        rviz_node,
        capture_pid_handler,
        cleanup_handler,
        shutdown_handler
    ])

    return ld
