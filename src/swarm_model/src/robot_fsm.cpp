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
: current_state_(State::FIND_OBJECT),
  object_detected_(false),
  distance_to_object_(100.0),      // Default far away
  at_home_(false),
  at_drop_(false),
  action_complete_(false),
  holding_object_(false),
  min_distance_to_grasp_(0.05)      // Default to 0.05 if not set
{
}

/**
 * @brief Update the state machine logic
 */
void RobotFSM::update()
{
  switch (current_state_) {
    case State::FIND_OBJECT:
      if (object_detected_) {
        current_state_ = State::APPROACH_OBJECT;
      }
      break;

    case State::APPROACH_OBJECT:
      // Logic: if dist <= min_dist, grasp
      if (distance_to_object_ <= min_distance_to_grasp_) {
        current_state_ = State::GRASP_OBJECT;
        action_complete_ = false;  // Reset for the new action
      }
      // Note: if object lost, maybe go back to SEARCH?
      // For now, adhere to simple diagram which transitions only on distance.
      // Ideally we check if object_detected_ is still true, but stick to diagram.
      break;

    case State::GRASP_OBJECT:
      if (action_complete_) {
        holding_object_ = true;
        current_state_ = State::MOVE_HOME;
      }
      break;

    case State::MOVE_HOME:
      if (at_home_) {
        if (holding_object_) {
          current_state_ = State::MOVE_OUT;
        } else {
          // If we came back home after releasing (or empty), go search again
          current_state_ = State::FIND_OBJECT;
        }
      }
      break;

    case State::MOVE_OUT:
      if (at_drop_) {
        current_state_ = State::RELEASE_OBJECT;
        action_complete_ = false;  // Reset for action
      }
      break;

    case State::RELEASE_OBJECT:
      if (action_complete_) {
        holding_object_ = false;
        current_state_ = State::MOVE_HOME;
      }
      break;
  }
}

/**
 * @brief Get the current state
 * @return State Current state
 */
State RobotFSM::get_state() const {return current_state_;}

/**
 * @brief Set object detected status
 * @param detected True if object is detected
 */
void RobotFSM::set_object_detected(bool detected) {object_detected_ = detected;}

/**
 * @brief Set distance to object
 * @param distance Distance in meters
 */
void RobotFSM::set_distance_to_object(double distance) {distance_to_object_ = distance;}

/**
 * @brief Set at home status
 * @param at_home True if robot is at home
 */
void RobotFSM::set_at_home(bool at_home) {at_home_ = at_home;}

/**
 * @brief Set at drop zone status
 * @param at_drop True if robot is at drop zone
 */
void RobotFSM::set_at_drop(bool at_drop) {at_drop_ = at_drop;}

/**
 * @brief Set action complete status
 * @param complete True if action (grasp/release) is complete
 */
void RobotFSM::set_action_complete(bool complete) {action_complete_ = complete;}

/**
 * @brief Set minimum distance to grasp object
 * @param distance Distance in meters
 */
void RobotFSM::set_min_distance_to_grasp(double distance) {min_distance_to_grasp_ = distance;}

}  // namespace swarm_model
