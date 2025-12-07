#ifndef SWARM_CONTROLLER_ROBOT_CONTROLLER_HPP_
#define SWARM_CONTROLLER_ROBOT_CONTROLLER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <swarm_model/robot_fsm.hpp>
#include <vector>

namespace swarm_controller
{

/**
 * @brief Parameters for initializing a Robot Controller
 */
struct RobotParams
{
  std::string robot_name;
  double rotation_speed;
  double distance_to_grasp_object;
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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Time last_image_time_;
  std::vector<uint8_t> center_pixels_;
  std::mutex mutex_;
  double rotation_speed_;
  double distance_to_grasp_object_;
  double current_distance_ = 100.0;  // Store latest lidar distance

  void scan_environment();
  void rotate_search_object();
  void approach_object();
  void move_to_home();
  void move_to_out();
  void grasp_object();
  void release_object();

  void move_robot(double linear_x, double angular_z);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_loop();
};

}  // namespace swarm_controller

#endif  // SWARM_CONTROLLER_ROBOT_CONTROLLER_HPP_
