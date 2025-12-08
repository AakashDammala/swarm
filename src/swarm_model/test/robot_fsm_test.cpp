/**
 * @file robot_fsm_test.cpp
 * @brief Unit tests for RobotFSM
 */

#include "swarm_model/robot_fsm.hpp"

#include <gtest/gtest.h>

// Test Case 1: Initial State
TEST(RobotFSMTest, InitialState)
{
  swarm_model::RobotFSM fsm;
  EXPECT_EQ(fsm.get_state(), swarm_model::State::FIND_OBJECT);
}

// Test Case 2: Find to Approach
TEST(RobotFSMTest, TransFindToApproach)
{
  swarm_model::RobotFSM fsm;

  // Initially finding
  ASSERT_EQ(fsm.get_state(), swarm_model::State::FIND_OBJECT);

  // Set input
  fsm.set_object_detected(true);
  fsm.update();

  EXPECT_EQ(fsm.get_state(), swarm_model::State::APPROACH_OBJECT);
}

// Test Case 3: Approach to Grasp
TEST(RobotFSMTest, TransApproachToGrasp)
{
  swarm_model::RobotFSM fsm;

  // Force state to Approach (by creating valid transition)
  fsm.set_object_detected(true);
  fsm.update();
  ASSERT_EQ(fsm.get_state(), swarm_model::State::APPROACH_OBJECT);

  // Set input: distance close enough
  // Default min grasp distance is 0.05
  fsm.set_distance_to_object(0.04);
  fsm.update();

  EXPECT_EQ(fsm.get_state(), swarm_model::State::GRASP_OBJECT);
}
