#ifndef SWARM_MODEL_ROBOT_FSM_HPP_
#define SWARM_MODEL_ROBOT_FSM_HPP_

/**
 * @file robot_fsm.hpp
 * @brief Header file for the Robot Finite State Machine
 */

#include <iostream>

namespace swarm_model
{

/**
 * @brief Robot State Enum
 */
enum class State
{
  FIND_OBJECT,      ///< Search for the object
  APPROACH_OBJECT,  ///< Move towards the object
  GRASP_OBJECT,     ///< Grasp the object
  MOVE_HOME,        ///< Return to home position
  MOVE_OUT,         ///< Move to drop-off location
  RELEASE_OBJECT    ///< Release the object
};

/**
 * @brief Robot Finite State Machine Class
 */
class RobotFSM
{
 public:
  /**
   * @brief Construct a new RobotFSM object
   */
  RobotFSM();

  /**
   * @brief Update the state machine
   */
  void update();

  /**
   * @brief Get the current state
   * @return State Current state
   */
  State get_state() const;

  // Inputs from sensors (setters)

  /**
   * @brief Set object detected status
   * @param detected True if object is detected
   */
  void set_object_detected(bool detected);

  /**
   * @brief Set distance to object
   * @param distance Distance to object in meters
   */
  void set_distance_to_object(double distance);

  /**
   * @brief Set at home status
   * @param at_home True if robot is at home
   */
  void set_at_home(bool at_home);

  /**
   * @brief Set at drop status
   * @param at_drop True if robot is at drop zone
   */
  void set_at_drop(bool at_drop);

  /**
   * @brief Set action complete status
   * @param complete True if action is complete
   */
  void set_action_complete(bool complete);  // Generic flag for discrete actions (grasp, release)

  /**
   * @brief Set minimum distance to grasp object
   * @param distance Minimum distance in meters
   */
  void set_min_distance_to_grasp(double distance);  // Setter for grasp threshold

 private:
  State current_state_;
  bool object_detected_;
  double distance_to_object_;
  bool at_home_;
  bool at_drop_;
  bool action_complete_;
  bool holding_object_;           // Internal state to track if we have the object
  double min_distance_to_grasp_;  // Threshold distance

  // Remove individual behavior methods as they are handled in update() state transitions or
  // Controller
};

}  // namespace swarm_model

#endif  // SWARM_MODEL_ROBOT_FSM_HPP_
