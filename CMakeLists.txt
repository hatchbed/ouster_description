cmake_minimum_required(VERSION 3.8)
project(ouster_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  meshes
  rviz
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  CONTRIBUTING.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_urdf_loading tests/test_urdf_loading.py)

  find_package(gz_common_vendor QUIET)
  find_package(gz-common5 QUIET COMPONENTS graphics)
  find_package(ament_index_cpp QUIET)
  if(gz-common5_FOUND AND ament_index_cpp_FOUND)
    ament_add_gtest(test_mesh_loading tests/test_mesh_loading.cpp)
    target_link_libraries(test_mesh_loading
      gz-common5::graphics
      ament_index_cpp::ament_index_cpp
    )
  endif()

  add_launch_test(tests/os1_gazebo_test.py)
endif()

ament_package()
