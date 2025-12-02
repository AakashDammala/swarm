#ifndef SWARM_VIEW_ROBOT_VIEW_HPP_
#define SWARM_VIEW_ROBOT_VIEW_HPP_

/**
 * @file robot_view.hpp
 * @brief Header file for the Robot View Node
 */

#include <rclcpp/rclcpp.hpp>

namespace swarm_view
{

/**
 * @brief Robot View Node
 *
 * Handles ROS 2 interface for sensors and actuators.
 */
class RobotView : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Robot View object
   */
  RobotView();

  /**
   * @brief Scan the environment
   */
  void scan_environment();

  /**
   * @brief Detect objects using sensors
   */
  void detect_object();

  /**
   * @brief Move the robot
   */
  void move_robot();

private:
  // ROS 2 publishers/subscribers would go here
};

}  // namespace swarm_view

#endif  // SWARM_VIEW_ROBOT_VIEW_HPP_
