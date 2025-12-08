cmake_minimum_required(VERSION 3.8)
project(swarm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_model REQUIRED)
find_package(swarm_view REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(swarm_controller_lib SHARED src/robot_controller.cpp)
ament_target_dependencies(swarm_controller_lib
  rclcpp
  swarm_model
  swarm_view
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs)

add_executable(robot_controller src/robot_controller_node.cpp)
target_link_libraries(robot_controller swarm_controller_lib)
ament_target_dependencies(robot_controller
  rclcpp
  swarm_model
  swarm_view
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_controller test/test_robot_controller.cpp)
  target_link_libraries(test_robot_controller swarm_controller_lib)
endif()

install(TARGETS
  robot_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

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

ament_package()
