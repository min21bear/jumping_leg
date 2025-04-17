cmake_minimum_required(VERSION 3.8)
project(jumping_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(jump_node src/jump_node.cpp)
ament_target_dependencies(jump_node
  rclcpp
  control_msgs
  gazebo_msgs
  std_msgs
)

add_executable(jump_node_hz src/jump_node_hz.cpp)
ament_target_dependencies(jump_node_hz
  rclcpp
  control_msgs
  gazebo_msgs
  std_msgs
)

add_executable(write_and_pub src/write_and_pub.cpp)
ament_target_dependencies(write_and_pub
  rclcpp
  control_msgs
  gazebo_msgs
  std_msgs
)


install(TARGETS
  jump_node
  jump_node_hz
  write_and_pub
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch urdf config mesh 
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
