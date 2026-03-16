ouster_description
===================

This is a ros2/ament conversion of the ros1/catkin ouster_description package originally from https://github.com/ouster-lidar/ouster_example

URDF descriptions for the OS0-32, OS1-32, and OS1-64 sensors are provided including sensor simulation configurations for gazebo.

Example use
------------

    <xacro:include filename="$(find ouster_description)/urdf/OS1-32.urdf.xacro"/>
    <xacro:OS1-32 parent="base_link" name="os1_sensor" hz="10" samples="220">
      <origin xyz="0 0 0.25" rpy="0 0 0" />
    </xacro:OS1-32>

