#ifndef DIFF_DRIVE__DIFF_DRIVE_NODE_HPP_
#define DIFF_DRIVE__DIFF_DRIVE_NODE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/logging.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"

#include "ros2_canopen_diff_drive/odometry.hpp"
#include "ros2_canopen_diff_drive/speed_limiter.hpp"
#include "ros2_canopen_diff_drive/visibility_control.hpp"

namespace diff_drive
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DiffDriveNode : public rclcpp_lifecycle::LifecycleNode
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  DIFF_DRIVE_PUBLIC
  DiffDriveNode(const std::string& node_name);

  DIFF_DRIVE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  DIFF_DRIVE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  DIFF_DRIVE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  DIFF_DRIVE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  DIFF_DRIVE_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  DIFF_DRIVE_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

protected:
  double left_wheel_position_transmission_ = 1.0;
  double right_wheel_position_transmission_ = 1.0;
  double left_wheel_velocity_transmission_ = 1.0;
  double right_wheel_velocity_transmission_ = 1.0;

  struct WheelParams
  {
    double separation = 0.0;  // w.r.t. the midpoint of the wheel width
    double radius = 0.0;      // Assumed to be the same for both wheels
    double separation_multiplier = 1.0;
    double left_radius_multiplier = 1.0;
    double right_radius_multiplier = 1.0;
  } wheel_params_;

  struct OdometryParams
  {
    bool open_loop = false;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_ =
      nullptr;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{ 500 };

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_wheel_init_client_ = nullptr;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_wheel_mode_client_ = nullptr;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_wheel_halt_client_ = nullptr;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_wheel_recover_client_ = nullptr;
  rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedPtr left_wheel_target_client_ = nullptr;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_wheel_init_client_ = nullptr;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_wheel_mode_client_ = nullptr;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_wheel_halt_client_ = nullptr;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_wheel_recover_client_ = nullptr;
  rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedPtr right_wheel_target_client_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_position_subscriber_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_position_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<std_msgs::msg::Float64>> left_wheel_position_ptr_{ nullptr };
  realtime_tools::RealtimeBox<std::shared_ptr<std_msgs::msg::Float64>> right_wheel_position_ptr_{ nullptr };

  rclcpp::Subscription<Twist>::SharedPtr velocity_command_stamped_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{ nullptr };

  std::queue<Twist> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ = nullptr;

  rclcpp::Time previous_update_timestamp_{ 0 };

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_{ 0, 0 };
  rclcpp::Time previous_publish_timestamp_{ 0 };

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  void update();
  bool reset();

  std::shared_ptr<rclcpp::TimerBase> timer_;
};
}  // namespace diff_drive
#endif  // DIFF_DRIVE__DIFF_DRIVE_NODE_HPP_
