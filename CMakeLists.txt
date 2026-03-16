cmake_minimum_required(VERSION 3.8)
project(ouster_description)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

install(DIRECTORY 
  config
  launch 
  meshes 
  rviz 
  urdf 
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_launch_test(tests/os1_gazebo_test.py)
endif()

ament_package()
