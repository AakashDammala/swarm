/**
 * @file robot_controller.cpp
 * @brief Implementation of the Robot Controller Node
 */
#include "swarm_controller/robot_controller.hpp"

namespace swarm_controller
{

RobotController::RobotController(const RobotParams& params)
    : Node("robot_controller",
           params.robot_name)  // Node name="robot_controller", Namespace=params.robot_name
{
  RCLCPP_INFO(this->get_logger(), "RobotController Initialized for %s", params.robot_name.c_str());

  last_image_time_ = this->now();

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

  // Timer to update FSM
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&RobotController::control_loop, this));
}

void RobotController::scan_environment()
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Scanning environment...");
  // Rotate to scan
  move_robot(0.0, 0.5);
}

void RobotController::detect_object()
{
  // Stub: In real implementation, check sensor data
  // For now, we just simulate detection randomly or via logic
  // fsm_.set_object_detected(true/false);
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
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_image_time_).seconds();
  if (dt > 0.0)
  {
    double freq = 1.0 / dt;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Receiving images at %.2f Hz", freq);
  }
  last_image_time_ = current_time;

  // Log image size
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received image of size %dx%d",
                       msg->width, msg->height);

  // Extract center 3x3 pixels
  // Center is at (width/2, height/2)
  // We want to extract pixels from [cy-1, cx-1] to [cy+1, cx+1]
  int cx = msg->width / 2;
  int cy = msg->height / 2;

  std::vector<uint8_t> new_center_pixels;
  // Step size is usually width * bytes_per_pixel. msg->step is bytes per row.
  int step = msg->step;
  // User specified to assume bgra8 format
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

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "Saved center pixels of size %lu", center_pixels_.size());
}

void RobotController::control_loop()
{
  // 1. Get inputs (Detect objects)
  detect_object();

  // 2. Update Model
  fsm_.update();

  // 3. Act based on State
  swarm_model::State current_state = fsm_.get_state();
  switch (current_state)
  {
    case swarm_model::State::SEARCHING:
      scan_environment();
      break;
    case swarm_model::State::COLLECTING:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Controller: Collecting...");
      move_robot(0.2, 0.0);  // Move forward to collect
      break;
    case swarm_model::State::DUMPING:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Controller: Dumping...");
      move_robot(0.0, 0.0);  // Stop to dump
      break;
    case swarm_model::State::AVOIDING:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Controller: Avoiding...");
      move_robot(-0.1, 0.5);  // Back up and turn
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
    // Robot names are usually 1-indexed in this project based on launch file
    params.robot_name = "robot_" + std::to_string(i + 1);

    auto node = std::make_shared<swarm_controller::RobotController>(params);
    nodes.push_back(node);
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
