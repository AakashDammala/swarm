#ifndef SWARM_MODEL_ROBOT_FSM_HPP_
#define SWARM_MODEL_ROBOT_FSM_HPP_

/**
 * @file robot_fsm.hpp
 * @brief Header file for the Robot Finite State Machine
 */

#include <iostream>

namespace swarm_model
{

enum class State
{
  FIND_OBJECT,
  APPROACH_OBJECT,
  GRASP_OBJECT,
  MOVE_HOME,
  MOVE_OUT,
  RELEASE_OBJECT
};

class RobotFSM
{
public:
  RobotFSM();
  void update();
  State get_state() const;

  // Inputs from sensors (setters)
  void set_object_detected(bool detected);
  void set_distance_to_object(double distance);
  void set_at_home(bool at_home);
  void set_at_drop(bool at_drop);
  void set_action_complete(bool complete);  // Generic flag for discrete actions (grasp, release)
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
