/**
 * @file robot_controller.cpp
 * @brief Implementation of the Robot Controller Node
 */
#include "swarm_controller/robot_controller.hpp"

namespace swarm_controller
{

RobotController::RobotController(const RobotParams& params)
    : Node("robot_controller",
           params.robot_name)  // Node name="robot_controller", Namespace=params.robot_name
{
  RCLCPP_INFO(this->get_logger(), "RobotController Initialized for %s", params.robot_name.c_str());

  // Publisher for velocity commands
  // Since we are in the namespace of the robot (e.g., /robot_1), publishing to "cmd_vel" ends up as
  // "/robot_1/cmd_vel"
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Timer to update FSM
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&RobotController::control_loop, this));
}

void RobotController::scan_environment()
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Scanning environment...");
  // Rotate to scan
  move_robot(0.0, 0.5);
}

void RobotController::detect_object()
{
  // Stub: In real implementation, check sensor data
  // For now, we just simulate detection randomly or via logic
  // fsm_.set_object_detected(true/false);
}

void RobotController::move_robot(double linear_x, double angular_z)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  cmd_vel_pub_->publish(msg);
}

void RobotController::control_loop()
{
  // 1. Get inputs (Detect objects)
  detect_object();

  // 2. Update Model
  fsm_.update();

  // 3. Act based on State
  swarm_model::State current_state = fsm_.get_state();
  switch (current_state)
  {
    case swarm_model::State::SEARCHING:
      scan_environment();
      break;
    case swarm_model::State::COLLECTING:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Controller: Collecting...");
      move_robot(0.2, 0.0);  // Move forward to collect
      break;
    case swarm_model::State::DUMPING:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Controller: Dumping...");
      move_robot(0.0, 0.0);  // Stop to dump
      break;
    case swarm_model::State::AVOIDING:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Controller: Avoiding...");
      move_robot(-0.1, 0.5);  // Back up and turn
      break;
    default:
      move_robot(0.0, 0.0);
      break;
  }
}

}  // namespace swarm_controller

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
    // Robot names are usually 1-indexed in this project based on launch file
    params.robot_name = "robot_" + std::to_string(i + 1);

    auto node = std::make_shared<swarm_controller::RobotController>(params);
    nodes.push_back(node);
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
