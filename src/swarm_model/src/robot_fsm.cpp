/**
 * @file robot_fsm.cpp
 * @brief Implementation of the Robot Finite State Machine
 */
#include "swarm_model/robot_fsm.hpp"

namespace swarm_model
{

/**
 * @brief Construct a new RobotFSM object
 */
RobotFSM::RobotFSM()
    : current_state_(State::IDLE), object_detected_(false), obstacle_detected_(false)
{
}

/**
 * @brief Update the state machine logic
 */
void RobotFSM::update()
{
  switch (current_state_)
  {
    case State::IDLE:
      current_state_ = State::SEARCHING;
      break;
    case State::SEARCHING:
      search();
      break;
    case State::COLLECTING:
      collect();
      break;
    case State::DUMPING:
      dump();
      break;
    case State::AVOIDING:
      avoid();
      break;
  }
}

/**
 * @brief Get the current state
 * @return State Current state
 */
State RobotFSM::get_state() const { return current_state_; }

/**
 * @brief Set object detected status
 * @param detected True if object is detected
 */
void RobotFSM::set_object_detected(bool detected) { object_detected_ = detected; }

/**
 * @brief Set obstacle detected status
 * @param detected True if obstacle is detected
 */
void RobotFSM::set_obstacle_detected(bool detected) { obstacle_detected_ = detected; }

/**
 * @brief Search behavior logic
 */
void RobotFSM::search()
{
  // Pure logic: if object detected, switch to collecting
  if (object_detected_)
  {
    current_state_ = State::COLLECTING;
  }
  else if (obstacle_detected_)
  {
    current_state_ = State::AVOIDING;
  }
}

/**
 * @brief Collect behavior logic
 */
void RobotFSM::collect()
{
  // Logic to transition to DUMPING (simplified for stub)
  // In real logic, this would depend on whether we grabbed it
  current_state_ = State::DUMPING;
}

/**
 * @brief Dump behavior logic
 */
void RobotFSM::dump()
{
  // Logic to transition to SEARCHING
  current_state_ = State::SEARCHING;
}

/**
 * @brief Avoid behavior logic
 */
void RobotFSM::avoid()
{
  // Logic to transition back to previous state or SEARCHING
  if (!obstacle_detected_)
  {
    current_state_ = State::SEARCHING;
  }
}

}  // namespace swarm_model
