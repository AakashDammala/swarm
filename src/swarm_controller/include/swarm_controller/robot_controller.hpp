#ifndef SWARM_CONTROLLER_ROBOT_CONTROLLER_HPP_
#define SWARM_CONTROLLER_ROBOT_CONTROLLER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <swarm_model/robot_fsm.hpp>

namespace swarm_controller
{

/**
 * @brief Parameters for initializing a Robot Controller
 */
struct RobotParams
{
  std::string robot_name;
};

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
   *
   * @param params Configuration parameters for the robot
   */
  explicit RobotController(const RobotParams& params);

 private:
  swarm_model::RobotFSM fsm_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void scan_environment();
  void detect_object();
  void move_robot(double linear_x, double angular_z);
  void control_loop();
};

}  // namespace swarm_controller

#endif  // SWARM_CONTROLLER_ROBOT_CONTROLLER_HPP_
