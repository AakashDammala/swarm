/**
 * @file robot_controller.cpp
 * @brief Implementation of the Robot Controller Node
 */
#include "swarm_controller/robot_controller.hpp"

#include <cmath>

namespace swarm_controller
{

std::string state_to_string(swarm_model::State state)
{
  switch (state)
  {
    case swarm_model::State::FIND_OBJECT:
      return "find_object";
    case swarm_model::State::APPROACH_OBJECT:
      return "approach_object";
    case swarm_model::State::GRASP_OBJECT:
      return "grasp_object";
    case swarm_model::State::MOVE_HOME:
      return "move_home";
    case swarm_model::State::MOVE_OUT:
      return "move_out";
    case swarm_model::State::RELEASE_OBJECT:
      return "release_object";
    default:
      return "unknown";
  }
}

RobotController::RobotController(const RobotParams& params)
    : rclcpp::Node("robot_controller", params.robot_name),
      robot_name_(params.robot_name),
      rotation_speed_(params.rotation_speed),
      distance_to_grasp_object_(params.distance_to_grasp_object)
{
  RCLCPP_INFO(this->get_logger(), "RobotController Initialized for %s", params.robot_name.c_str());

  // Set the FSM parameter
  fsm_.set_min_distance_to_grasp(distance_to_grasp_object_);

  // last_image_time_ = this->now();

  // Publisher for velocity commands
  // Since we are in the namespace of the robot (e.g., /robot_1), publishing to "cmd_vel" ends up as
  // "/robot_1/cmd_vel"
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Subscribe to camera image
  // Topic: /robot_X/robot_X/camera/image_color (based on user request)
  // Since we are in /robot_X namespace, we subscribe to "robot_X/camera/image_color"
  std::string camera_topic = params.robot_name + "/camera/image_color";
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic, 8, std::bind(&RobotController::image_callback, this, std::placeholders::_1));

  // Subscribe to Lidar
  std::string scan_topic = params.robot_name + "/lidar";
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 10, std::bind(&RobotController::lidar_callback, this, std::placeholders::_1));

  // Initialize TF Listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // run the timer callback at 20 Hz
  timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                   std::bind(&RobotController::control_loop, this));
}

void RobotController::scan_environment()
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Scanning environment...");
  move_robot(0.0, 0.5);
}

void RobotController::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float min_dist = 100.0;
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    if (min_dist > msg->ranges[i])
    {
      min_dist = msg->ranges[i];
    }
  }

  RCLCPP_INFO(this->get_logger(), "Minimum distance: %.2f", min_dist);

  current_distance_ = min_dist;
  fsm_.set_distance_to_object(current_distance_);
}

void RobotController::rotate_search_object()
{
  // "send -0.05 omega cmd_vel"

  bool found_green = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!center_pixels_.empty())
    {
      for (size_t i = 0; i < center_pixels_.size(); i += 3)
      {
        if (i + 2 < center_pixels_.size())
        {
          uint8_t r = center_pixels_[i];
          uint8_t g = center_pixels_[i + 1];
          uint8_t b = center_pixels_[i + 2];

          if (g > 200 && r < 50 && b < 50)
          {
            found_green = true;
            break;
          }
        }
      }
    }
  }

  fsm_.set_object_detected(found_green);

  // If found, FSM will switch state next loop.
  // Diagram says "not found -> rotate".
  if (!found_green)
  {
    move_robot(0.0, -0.05);  // -0.05 omega
  }
  else
  {
    move_robot(0.0, 0.0);  // Stop momentarily? Or just let next state handle it.
  }
}

void RobotController::approach_object()
{
  // "send 0.05 lin_vel till lidar distance < 0.05 m"
  move_robot(0.05, 0.0);
}

void RobotController::grasp_object()
{
  RCLCPP_INFO(this->get_logger(), "Grasping object...");
  move_robot(0.0, 0.0);  // Stop

  // Simulate release time
  rclcpp::sleep_for(std::chrono::seconds(2));

  fsm_.set_action_complete(true);
}

void RobotController::move_to_home()
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Moving Home...");

  geometry_msgs::msg::Pose current_pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose = robot_pose_;
  }

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 0.0;
  goal_pose.position.y = 0.0;
  goal_pose.orientation.w = 1.0;

  auto cmd_vel = move_to_location(current_pose, goal_pose);

  if (cmd_vel.linear.x == 0.0 && cmd_vel.angular.z == 0.0)
  {
    fsm_.set_at_home(true);
    move_robot(0.0, 0.0);
  }
  else
  {
    fsm_.set_at_home(false);
    cmd_vel_pub_->publish(cmd_vel);
  }
}

void RobotController::move_to_out()
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Moving Out...");

  geometry_msgs::msg::Pose current_pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose = robot_pose_;
  }

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 0.7;
  goal_pose.position.y = 0.0;
  goal_pose.orientation.w = 1.0;

  auto cmd_vel = move_to_location(current_pose, goal_pose);

  if (cmd_vel.linear.x == 0.0 && cmd_vel.angular.z == 0.0)
  {
    fsm_.set_at_drop(true);
    move_robot(0.0, 0.0);
  }
  else
  {
    fsm_.set_at_drop(false);
    cmd_vel_pub_->publish(cmd_vel);
  }
}

void RobotController::release_object()
{
  RCLCPP_INFO(this->get_logger(), "Releasing object...");

  // Simulate release time
  rclcpp::sleep_for(std::chrono::seconds(2));

  move_robot(0.0, 0.0);
  fsm_.set_action_complete(true);
}

geometry_msgs::msg::Twist RobotController::move_to_location(
    const geometry_msgs::msg::Pose& current_pose, const geometry_msgs::msg::Pose& goal_pose)
{
  geometry_msgs::msg::Twist cmd_vel;

  // Calculate distance
  double dx = goal_pose.position.x - current_pose.position.x;
  double dy = goal_pose.position.y - current_pose.position.y;
  double dist = std::hypot(dx, dy);

  // Check if reached (3cm threshold)
  if (dist < 0.03)
  {
    return cmd_vel;  // Zero initialized
  }

  // Calculate target yaw
  double target_yaw = std::atan2(dy, dx);

  // Get current yaw
  tf2::Quaternion q;
  tf2::fromMsg(current_pose.orientation, q);
  double current_yaw = tf2::impl::getYaw(q);

  // Calculate yaw error
  double yaw_error = target_yaw - current_yaw;
  // Normalize to [-pi, pi]
  while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

  // Control logic: Rotate first, then move straight
  // Threshold for rotation alignment: 0.1 rad (~5.7 degrees)
  if (std::abs(yaw_error) > 0.1)
  {
    // Rotate
    // P-controller for rotation
    cmd_vel.angular.z = 1.0 * yaw_error;
    // Clamp angular velocity
    if (cmd_vel.angular.z > 0.2) cmd_vel.angular.z = 0.2;
    if (cmd_vel.angular.z < -0.2) cmd_vel.angular.z = -0.2;
  }
  else
  {
    // Move straight
    cmd_vel.linear.x = 0.1;  // Safe constant speed
  }

  return cmd_vel;
}

void RobotController::move_robot(double linear_x, double angular_z)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  cmd_vel_pub_->publish(msg);
}

void RobotController::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // rclcpp::Time current_time = this->now();
  // double dt = (current_time - last_image_time_).seconds();
  // if (dt > 0.0)
  // {
  //   double freq = 1.0 / dt;
  //   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
  //                        "Receiving images at %.2f Hz", freq);
  // }
  // last_image_time_ = current_time;

  // Log image size
  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received image of size
  // %dx%d",
  //                      msg->width, msg->height);

  // Extract center 3x3 pixels
  // Center is at (width/2, height/2)
  // We want to extract pixels from [cy-1, cx-1] to [cy+1, cx+1]
  int cx = msg->width / 2;
  int cy = msg->height / 2;

  std::vector<uint8_t> new_center_pixels;
  // Step size is usually width * bytes_per_pixel. msg->step is bytes per row.
  int step = msg->step;
  // Assume bgra8 format
  int bytes_per_pixel = 4;

  // Store the center 3 pixels for further processing
  for (int y = cy - 1; y <= cy + 1; ++y)
  {
    for (int x = cx - 1; x <= cx + 1; ++x)
    {
      int pixel_idx = y * step + x * bytes_per_pixel;

      // Safety check
      if (pixel_idx + 3 < (int)msg->data.size())
      {
        // bgra8 encoding: Blue, Green, Red, Alpha
        uint8_t blue = msg->data[pixel_idx];
        uint8_t green = msg->data[pixel_idx + 1];
        uint8_t red = msg->data[pixel_idx + 2];
        // uint8_t alpha = msg->data[pixel_idx + 3]; // Skip alpha

        // Store as RGB
        new_center_pixels.push_back(red);
        new_center_pixels.push_back(green);
        new_center_pixels.push_back(blue);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    center_pixels_ = new_center_pixels;
  }

  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
  //                      "Saved center pixels of size %lu", center_pixels_.size());
}

void RobotController::control_loop()
{
  // Get the robot's pose, from it's home frame to base_link frame
  try
  {
    geometry_msgs::msg::TransformStamped t;
    t = tf_buffer_->lookupTransform(robot_name_ + "/home", robot_name_ + "/base_link",
                                    tf2::TimePointZero);

    std::lock_guard<std::mutex> lock(pose_mutex_);
    robot_pose_.position.x = t.transform.translation.x;
    robot_pose_.position.y = t.transform.translation.y;
    robot_pose_.position.z = t.transform.translation.z;
    robot_pose_.orientation = t.transform.rotation;

    // RCLCPP_INFO(this->get_logger(), "Robot pose: x=%.2f, y=%.2f", t.transform.translation.x,
    //             t.transform.translation.y);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Could not transform %s/base_link to %s/home: %s", robot_name_.c_str(),
                         robot_name_.c_str(), ex.what());
  }

  fsm_.update();

  swarm_model::State current_state = fsm_.get_state();

  double yaw = 0.0;
  double x = 0.0;
  double y = 0.0;

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    x = robot_pose_.position.x;
    y = robot_pose_.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(robot_pose_.orientation, q);
    yaw = tf2::impl::getYaw(q);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "[%s] State: %s | Pos: (%.2f, %.2f) | Yaw: %.2f | Dist: %.2f",
                       robot_name_.c_str(), state_to_string(current_state).c_str(), x, y, yaw,
                       current_distance_);

  switch (current_state)
  {
    case swarm_model::State::FIND_OBJECT:
      rotate_search_object();
      break;
    case swarm_model::State::APPROACH_OBJECT:
      approach_object();
      break;
    case swarm_model::State::GRASP_OBJECT:
      grasp_object();
      break;
    case swarm_model::State::MOVE_HOME:
      move_to_home();
      break;
    case swarm_model::State::MOVE_OUT:
      move_to_out();
      break;
    case swarm_model::State::RELEASE_OBJECT:
      release_object();
      break;
    default:
      move_robot(0.0, 0.0);
      break;
  }
}

}  // namespace swarm_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<std::shared_ptr<swarm_controller::RobotController>> nodes;

  // Create a temporary node to read parameters from launch file
  auto param_node = std::make_shared<rclcpp::Node>("local_spawner");
  param_node->declare_parameter("num_robots", 2);
  int num_robots = param_node->get_parameter("num_robots").as_int();

  RCLCPP_INFO(param_node->get_logger(), "Spawning %d robot controllers...", num_robots);

  for (int i = 0; i < num_robots; ++i)
  {
    swarm_controller::RobotParams params;

    // Robot names are 1-indexed in this project
    params.robot_name = "robot_" + std::to_string(i + 1);
    params.rotation_speed = 0.1;
    params.distance_to_grasp_object = 0.35;

    auto node = std::make_shared<swarm_controller::RobotController>(params);
    nodes.push_back(node);
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
