cmake_minimum_required(VERSION 3.8)
project(controll_with_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(simple_motion_node src/simple_motion_node.cpp)
add_executable(command2motion_node src/command2motion_node.cpp)
ament_target_dependencies(
    simple_motion_node
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
)
ament_target_dependencies(
    command2motion_node
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
)

install(TARGETS 
  command2motion_node
  simple_motion_node 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
