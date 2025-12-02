// File to mock rclcpp logging for doxygen generation
// This is for removing un-necessary steps while generating UML diagrams
#include <rclcpp/logging.hpp>

#undef RCLCPP_INFO
#define RCLCPP_INFO(...) (void)0

#undef RCLCPP_INFO_THROTTLE
#define RCLCPP_INFO_THROTTLE(...) (void)0

#undef RCLCPP_WARN
#define RCLCPP_WARN(...) (void)0

#undef RCLCPP_ERROR
#define RCLCPP_ERROR(...) (void)0

#undef RCLCPP_FATAL
#define RCLCPP_FATAL(...) (void)0
