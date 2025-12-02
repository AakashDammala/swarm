cmake_minimum_required(VERSION 3.8)
project(swarm_view)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(catch_ros2 REQUIRED)

include_directories(include)

add_executable(robot_view src/robot_view.cpp)
ament_target_dependencies(robot_view rclcpp)

install(TARGETS
  robot_view
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# Install directories
install(DIRECTORY 
  config 
  launch 
  resource 
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # the following line skips the linter which checks for uninitialized variables
  # ensure the variables are initialized
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Catch2 ROS2 integration test
  add_executable(robot_topics_test test/robot_topics_test.cpp)
  ament_target_dependencies(robot_topics_test rclcpp sensor_msgs geometry_msgs catch_ros2)
  add_test(NAME robot_topics_test COMMAND robot_topics_test)
endif()

ament_package()
