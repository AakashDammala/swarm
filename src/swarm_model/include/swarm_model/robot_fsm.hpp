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
  IDLE,
  SEARCHING,
  COLLECTING,
  DUMPING,
  AVOIDING
};

class RobotFSM
{
public:
  RobotFSM();
  void update();
  State get_state() const;

  // Inputs from sensors (setters)
  void set_object_detected(bool detected);
  void set_obstacle_detected(bool detected);

private:
  State current_state_;
  bool object_detected_;
  bool obstacle_detected_;

  void search();
  void collect();
  void dump();
  void avoid();
};

}  // namespace swarm_model

#endif  // SWARM_MODEL_ROBOT_FSM_HPP_
