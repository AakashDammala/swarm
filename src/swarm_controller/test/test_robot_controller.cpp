/**
 * @file test_robot_controller.cpp
 * @brief Unit tests for Robot Controller
 */
#include <gtest/gtest.h>

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

#include "swarm_controller/robot_controller.hpp"

TEST(RobotControllerTest, StateToString)
{
  EXPECT_EQ(swarm_controller::state_to_string(swarm_model::State::FIND_OBJECT), "find_object");
  EXPECT_EQ(swarm_controller::state_to_string(swarm_model::State::APPROACH_OBJECT),
            "approach_object");
  EXPECT_EQ(swarm_controller::state_to_string(swarm_model::State::GRASP_OBJECT), "grasp_object");
  EXPECT_EQ(swarm_controller::state_to_string(swarm_model::State::MOVE_HOME), "move_home");
  EXPECT_EQ(swarm_controller::state_to_string(swarm_model::State::MOVE_OUT), "move_out");
  EXPECT_EQ(swarm_controller::state_to_string(swarm_model::State::RELEASE_OBJECT),
            "release_object");
}

class RobotControllerTest : public ::testing::Test
{
 protected:
  // Helper to access the private static method
  static geometry_msgs::msg::Twist call_move_to_location(
      const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Pose& goal_pose)
  {
    // Accessing private static method via friend class or similar trick if needed,
    // but we made it public static in the header based on the plan.
    // Let's verify if I actually made it public in the previous step.
    // Yes, I moved it to the public section (actually, looking at the diff, I put it under a new
    // private section?) Wait, let's double check the previous tool call for robot_controller.hpp.

    // In step 42 (replace_file_content for robot_controller.hpp), I replaced:
    /*
      swarm_model::RobotFSM fsm_;
      ...
      void release_object();

      geometry_msgs::msg::Twist move_to_location(...)
    */
    // WITH:
    /*
      static geometry_msgs::msg::Twist move_to_location(...)

     private:
      swarm_model::RobotFSM fsm_;
    */
    // The `private:` label was already there before `swarm_model::RobotFSM fsm_` in the original
    // file? Let's re-read the file content of `robot_controller.hpp` to be sure where
    // `move_to_location` ended up. The original file had `private:` at line 48. I replaced from
    // line 49. So `move_to_location` was inserted *after* `private:` line 48. So it is PRIVATE
    // STATIC. My plan said "Make move_to_location a public static method". I made a mistake in the
    // `replace_file_content` call. I put it under the existing `private:` block. I need to fix this
    // in `robot_controller.hpp` to make it public.
    return swarm_controller::RobotController::move_to_location(current_pose, goal_pose);
  }
};

// Start defining tests assuming I will fix the visibility
TEST(RobotControllerTest, MoveToLocation_AlreadyThere)
{
  geometry_msgs::msg::Pose current;
  current.position.x = 1.0;
  current.position.y = 1.0;

  geometry_msgs::msg::Pose goal;
  goal.position.x = 1.001;  // Within 3cm
  goal.position.y = 1.001;

  auto cmd = swarm_controller::RobotController::move_to_location(current, goal);
  EXPECT_EQ(cmd.linear.x, 0.0);
  EXPECT_EQ(cmd.angular.z, 0.0);
}

TEST(RobotControllerTest, MoveToLocation_RotateOnly)
{
  geometry_msgs::msg::Pose current;
  current.position.x = 0.0;
  current.position.y = 0.0;
  // Facing East (0 rad)
  current.orientation.w = 1.0;
  current.orientation.z = 0.0;

  geometry_msgs::msg::Pose goal;
  goal.position.x = 0.0;
  goal.position.y = 1.0;  // North (Pi/2 rad)

  // Expected action: Rotate positive angular.z, zero linear.x
  auto cmd = swarm_controller::RobotController::move_to_location(current, goal);

  EXPECT_EQ(cmd.linear.x, 0.0);
  EXPECT_GT(cmd.angular.z, 0.0);
}

TEST(RobotControllerTest, MoveToLocation_MoveStraight)
{
  geometry_msgs::msg::Pose current;
  current.position.x = 0.0;
  current.position.y = 0.0;
  // Facing East (0 rad)
  current.orientation.w = 1.0;
  current.orientation.z = 0.0;

  geometry_msgs::msg::Pose goal;
  goal.position.x = 1.0;  // East
  goal.position.y = 0.0;

  // Expected action: Move positive linear.x, zero angular.z
  auto cmd = swarm_controller::RobotController::move_to_location(current, goal);

  EXPECT_GT(cmd.linear.x, 0.0);
  EXPECT_NEAR(cmd.angular.z, 0.0, 0.001);
}
