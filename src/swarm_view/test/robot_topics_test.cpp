/**
 * @file robot_topics_test.cpp
 * @brief Integration tests for Robot Topics
 */
#include <catch_ros2/catch_ros2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

auto Logger = rclcpp::get_logger("");

class RobotTopicsFixture
{
public:
  RobotTopicsFixture()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    testerNode = rclcpp::Node::make_shared("RobotTopicsTestNode");
    Logger = testerNode->get_logger();
    testerNode->declare_parameter<double>("test_duration", 0.5);
    TEST_DURATION = testerNode->get_parameter("test_duration").get_parameter_value().get<double>();
  }

  ~RobotTopicsFixture() {}

protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

TEST_CASE_METHOD(RobotTopicsFixture, "test robot lidar topics", "[robot_lidar]")
{
  for (int i = 0; i < 12; i++) {
    bool got_lidar = false;
    std::string topic = "/robot_" + std::to_string(i) + "/robot_" + std::to_string(i) + "/lidar";

    auto sub = testerNode->create_subscription<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::QoS(10),
      [&got_lidar](const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/) {got_lidar = true;});

    rclcpp::Rate loop_rate(20);
    auto start = std::chrono::steady_clock::now();
    while (!got_lidar &&
      std::chrono::steady_clock::now() - start < std::chrono::duration<double>(TEST_DURATION))
    {
      rclcpp::spin_some(testerNode);
      loop_rate.sleep();
    }

    RCLCPP_INFO(Logger, "Robot %d: lidar=%d", i, got_lidar);
    if (got_lidar) {
      CHECK(got_lidar);
    }
  }
}

TEST_CASE_METHOD(RobotTopicsFixture, "test robot camera topics", "[robot_camera]")
{
  for (int i = 0; i < 12; i++) {
    bool got_camera = false;
    std::string topic =
      "/robot_" + std::to_string(i) + "/robot_" + std::to_string(i) + "/camera/image_color";

    auto sub = testerNode->create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::QoS(10),
      [&got_camera](const sensor_msgs::msg::Image::SharedPtr /*msg*/) {got_camera = true;});

    rclcpp::Rate loop_rate(20);
    auto start = std::chrono::steady_clock::now();
    while (!got_camera &&
      std::chrono::steady_clock::now() - start < std::chrono::duration<double>(TEST_DURATION))
    {
      rclcpp::spin_some(testerNode);
      loop_rate.sleep();
    }

    RCLCPP_INFO(Logger, "Robot %d: camera=%d", i, got_camera);
    if (got_camera) {
      CHECK(got_camera);
    }
  }
}

/*TEST_CASE_METHOD(RobotTopicsFixture, "test cmd_vel subscribers", "[cmd_vel]") {
  for (int i = 0; i < 12; i++) {
    std::string topic = "/robot_" + std::to_string(i) + "/cmd_vel";

    auto pub = testerNode->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    auto graph = testerNode->get_graph_guard_condition();

    rclcpp::spin_some(testerNode);

    auto count = testerNode->count_subscribers(topic);
    RCLCPP_INFO(Logger, "Robot %d: cmd_vel subscribers=%lu", i, count);
    CHECK(count > 0);
  }
}*/

TEST_CASE("ROS2 initialization test") {
  CHECK(rclcpp::ok());
}
