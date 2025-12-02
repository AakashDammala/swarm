/**
 * @file robot_controller.cpp
 * @brief Implementation of the Robot Controller Node
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swarm_model/robot_fsm.hpp>

namespace swarm_controller
{

/**
 * @brief Robot Controller Node
 *
 * Orchestrates the robot behavior by using the RobotFSM model and communicating with ROS 2.
 */
class RobotController : public rclcpp::Node
{
 public:
  /**
   * @brief Construct a new Robot Controller object
   */
  RobotController() : Node("robot_controller")
  {
    RCLCPP_INFO(this->get_logger(), "RobotController Initialized");

    // Publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Timer to update FSM
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&RobotController::control_loop, this));
  }

 private:
  swarm_model::RobotFSM fsm_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void scan_environment()
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Scanning environment...");
    // Rotate to scan
    move_robot(0.0, 0.5);
  }

  void detect_object()
  {
    // Stub: In real implementation, check sensor data
    // For now, we just simulate detection randomly or via logic
    // fsm_.set_object_detected(true/false);
  }

  void move_robot(double linear_x, double angular_z)
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    cmd_vel_pub_->publish(msg);
  }

  void control_loop()
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
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Controller: Dumping...");
        move_robot(0.0, 0.0);  // Stop to dump
        break;
      case swarm_model::State::AVOIDING:
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Controller: Avoiding...");
        move_robot(-0.1, 0.5);  // Back up and turn
        break;
      default:
        move_robot(0.0, 0.0);
        break;
    }
  }
};

}  // namespace swarm_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<swarm_controller::RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
