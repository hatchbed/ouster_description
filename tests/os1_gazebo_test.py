#!/usr/bin/env python3
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
import time
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

# ---------------------------------------------------------
# 1. LAUNCH SETUP
# ---------------------------------------------------------


def generate_test_description():
    """Launch the world file before running the tests."""
    skip_gui = SetEnvironmentVariable('GZ_GUI_SKIP', '1')

    os1_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ouster_description'),
                'launch',
                'os1_world.launch.py'
            ])
        ),
        launch_arguments={
            'rviz': 'false',
            'gui': 'false',
        }.items()
    )

    return LaunchDescription([
        skip_gui,
        os1_world_launch,
        launch_testing.actions.ReadyToTest()
    ])


# Helper class to capture topic message timestamps
class MessageCollector:

    def __init__(self, node, topic_name, msg_type, qos=10):
        self.msgs = []
        self.sub = node.create_subscription(
            msg_type, topic_name, self.callback, qos)

    def callback(self, msg):
        self.msgs.append(time.time())


# ---------------------------------------------------------
# 2. TEST CASES (Replaces hztest, publishtest, paramtest)
# ---------------------------------------------------------
class TestOusterGazebo(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_ouster_gazebo_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_simulation_topics_and_params(self):
        points_col = MessageCollector(self.node, '/ouster/points', PointCloud2)
        imu_col = MessageCollector(self.node, '/ouster/imu', Imu)

        from rosgraph_msgs.msg import Clock
        clock_col = MessageCollector(self.node, '/clock', Clock)

        transient_local_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        robot_desc_col = MessageCollector(
            self.node, '/robot_description', String, qos=transient_local_qos)

        start_time = time.time()
        wait_timeout = 40.0

        points_recv, imu_recv, clock_recv, robot_desc_recv = False, False, False, False

        while time.time() - start_time < wait_timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if not points_recv and len(points_col.msgs) > 0:
                points_recv = True
            if not imu_recv and len(imu_col.msgs) > 0:
                imu_recv = True
            if not clock_recv and len(clock_col.msgs) > 0:
                clock_recv = True
            if not robot_desc_recv and len(robot_desc_col.msgs) > 0:
                robot_desc_recv = True

            # If all are True, break early
            if points_recv and imu_recv and clock_recv and robot_desc_recv:
                break

        self.assertTrue(robot_desc_recv, 'robot_description topic was not published')

        # Ensure topics are ready to be measured
        self.assertTrue(points_recv, 'No messages received on /ouster/points')
        self.assertTrue(imu_recv, 'No messages received on /ouster/imu')

        # ---------------------------------------------------------
        # Measure Hz for 10.0 seconds (hztest Phase)
        # ---------------------------------------------------------
        points_col.msgs.clear()
        imu_col.msgs.clear()

        hz_test_duration = 10.0
        start_hz_time = time.time()

        while time.time() - start_hz_time < hz_test_duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        points_hz = len(points_col.msgs) / hz_test_duration
        imu_hz = len(imu_col.msgs) / hz_test_duration

        self.assertGreaterEqual(points_hz, 1.0, f'/ouster/points Hz too low: {points_hz}')
        self.assertLessEqual(points_hz, 25.0, f'/ouster/points Hz too high: {points_hz}')

        self.assertGreaterEqual(imu_hz, 50.0, f'/ouster/imu Hz too low: {imu_hz}')
        self.assertLessEqual(imu_hz, 150.0, f'/ouster/imu Hz too high: {imu_hz}')
