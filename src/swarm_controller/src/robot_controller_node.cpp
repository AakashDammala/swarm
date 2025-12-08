/**
 * @file robot_controller_node.cpp
 * @brief Node entry point for Robot Controller
 */
#include <rclcpp/rclcpp.hpp>

#include "swarm_controller/robot_controller.hpp"

/**
 * @brief Main function
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<std::shared_ptr<swarm_controller::RobotController>> nodes;

  // Create a temporary node to read parameters from launch file
  auto param_node = std::make_shared<rclcpp::Node>("local_spawner");
  param_node->declare_parameter("num_robots", 2);
  int num_robots = param_node->get_parameter("num_robots").as_int();

  RCLCPP_INFO(param_node->get_logger(), "Spawning %d robot controllers...", num_robots);

  for (int i = 0; i < num_robots; ++i)
  {
    swarm_controller::RobotParams params;

    // Robot names are 1-indexed in this project
    params.robot_name = "robot_" + std::to_string(i + 1);
    params.rotation_speed = 0.3;
    params.distance_to_grasp_object = 0.35;

    auto node = std::make_shared<swarm_controller::RobotController>(params);
    nodes.push_back(node);
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
