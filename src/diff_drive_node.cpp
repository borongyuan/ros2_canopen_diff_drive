#include "ros2_canopen_diff_drive/diff_drive_node.hpp"
// #include "lifecycle_msgs/msg/state.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_STAMPED_TOPIC = "~/cmd_vel_stamped";
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace diff_drive
{
// using namespace std::chrono_literals;
// using lifecycle_msgs::msg::State;

DiffDriveNode::DiffDriveNode(const std::string& node_name) : rclcpp_lifecycle::LifecycleNode(node_name)
{
  this->declare_parameter<double>("left_wheel_position_transmission", left_wheel_position_transmission_);
  this->declare_parameter<double>("right_wheel_position_transmission", right_wheel_position_transmission_);
  this->declare_parameter<double>("left_wheel_velocity_transmission", left_wheel_velocity_transmission_);
  this->declare_parameter<double>("right_wheel_velocity_transmission", right_wheel_velocity_transmission_);

  this->declare_parameter<double>("wheel_separation", wheel_params_.separation);
  this->declare_parameter<double>("wheel_radius", wheel_params_.radius);
  this->declare_parameter<double>("wheel_separation_multiplier", wheel_params_.separation_multiplier);
  this->declare_parameter<double>("left_wheel_radius_multiplier", wheel_params_.left_radius_multiplier);
  this->declare_parameter<double>("right_wheel_radius_multiplier", wheel_params_.right_radius_multiplier);

  this->declare_parameter<std::string>("odom_frame_id", odom_params_.odom_frame_id);
  this->declare_parameter<std::string>("base_frame_id", odom_params_.base_frame_id);
  this->declare_parameter<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
  this->declare_parameter<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
  this->declare_parameter<bool>("open_loop", odom_params_.open_loop);
  this->declare_parameter<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

  this->declare_parameter<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
  this->declare_parameter<bool>("publish_limited_velocity", publish_limited_velocity_);
  this->declare_parameter<int>("velocity_rolling_window_size", 10);
  this->declare_parameter<bool>("use_stamped_vel", use_stamped_vel_);

  this->declare_parameter<bool>("linear.x.has_velocity_limits", false);
  this->declare_parameter<bool>("linear.x.has_acceleration_limits", false);
  this->declare_parameter<bool>("linear.x.has_jerk_limits", false);
  this->declare_parameter<double>("linear.x.max_velocity", NAN);
  this->declare_parameter<double>("linear.x.min_velocity", NAN);
  this->declare_parameter<double>("linear.x.max_acceleration", NAN);
  this->declare_parameter<double>("linear.x.min_acceleration", NAN);
  this->declare_parameter<double>("linear.x.max_jerk", NAN);
  this->declare_parameter<double>("linear.x.min_jerk", NAN);

  this->declare_parameter<bool>("angular.z.has_velocity_limits", false);
  this->declare_parameter<bool>("angular.z.has_acceleration_limits", false);
  this->declare_parameter<bool>("angular.z.has_jerk_limits", false);
  this->declare_parameter<double>("angular.z.max_velocity", NAN);
  this->declare_parameter<double>("angular.z.min_velocity", NAN);
  this->declare_parameter<double>("angular.z.max_acceleration", NAN);
  this->declare_parameter<double>("angular.z.min_acceleration", NAN);
  this->declare_parameter<double>("angular.z.max_jerk", NAN);
  this->declare_parameter<double>("angular.z.min_jerk", NAN);

  this->declare_parameter<double>("publish_rate", publish_rate_);
}

CallbackReturn DiffDriveNode::on_configure(const rclcpp_lifecycle::State&)
{
  auto logger = this->get_logger();

  // update parameters
  left_wheel_position_transmission_ = this->get_parameter("left_wheel_position_transmission").as_double();
  right_wheel_position_transmission_ = this->get_parameter("right_wheel_position_transmission").as_double();
  left_wheel_velocity_transmission_ = this->get_parameter("left_wheel_velocity_transmission").as_double();
  right_wheel_velocity_transmission_ = this->get_parameter("right_wheel_velocity_transmission").as_double();

  wheel_params_.separation = this->get_parameter("wheel_separation").as_double();
  wheel_params_.radius = this->get_parameter("wheel_radius").as_double();
  wheel_params_.separation_multiplier = this->get_parameter("wheel_separation_multiplier").as_double();
  wheel_params_.left_radius_multiplier = this->get_parameter("left_wheel_radius_multiplier").as_double();
  wheel_params_.right_radius_multiplier = this->get_parameter("right_wheel_radius_multiplier").as_double();

  const auto wheels = wheel_params_;

  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  odometry_.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
  odometry_.setVelocityRollingWindowSize(this->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = this->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = this->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = this->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = this->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = this->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = this->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ =
      std::chrono::milliseconds{ static_cast<int>(this->get_parameter("cmd_vel_timeout").as_double() * 1000.0) };
  publish_limited_velocity_ = this->get_parameter("publish_limited_velocity").as_bool();
  use_stamped_vel_ = this->get_parameter("use_stamped_vel").as_bool();

  try
  {
    limiter_linear_ = SpeedLimiter(this->get_parameter("linear.x.has_velocity_limits").as_bool(),
                                   this->get_parameter("linear.x.has_acceleration_limits").as_bool(),
                                   this->get_parameter("linear.x.has_jerk_limits").as_bool(),
                                   this->get_parameter("linear.x.min_velocity").as_double(),
                                   this->get_parameter("linear.x.max_velocity").as_double(),
                                   this->get_parameter("linear.x.min_acceleration").as_double(),
                                   this->get_parameter("linear.x.max_acceleration").as_double(),
                                   this->get_parameter("linear.x.min_jerk").as_double(),
                                   this->get_parameter("linear.x.max_jerk").as_double());
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(logger, "Error configuring linear speed limiter: %s", e.what());
  }

  try
  {
    limiter_angular_ = SpeedLimiter(this->get_parameter("angular.z.has_velocity_limits").as_bool(),
                                    this->get_parameter("angular.z.has_acceleration_limits").as_bool(),
                                    this->get_parameter("angular.z.has_jerk_limits").as_bool(),
                                    this->get_parameter("angular.z.min_velocity").as_double(),
                                    this->get_parameter("angular.z.max_velocity").as_double(),
                                    this->get_parameter("angular.z.min_acceleration").as_double(),
                                    this->get_parameter("angular.z.max_acceleration").as_double(),
                                    this->get_parameter("angular.z.min_jerk").as_double(),
                                    this->get_parameter("angular.z.max_jerk").as_double());
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(logger, "Error configuring angular speed limiter: %s", e.what());
  }

  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  left_wheel_init_client_ = this->create_client<std_srvs::srv::Trigger>("/left_wheel/init");
  left_wheel_mode_client_ = this->create_client<std_srvs::srv::Trigger>("/left_wheel/velocity_mode");
  left_wheel_halt_client_ = this->create_client<std_srvs::srv::Trigger>("/left_wheel/halt");
  left_wheel_recover_client_ = this->create_client<std_srvs::srv::Trigger>("/left_wheel/recover");
  left_wheel_target_client_ = this->create_client<canopen_interfaces::srv::COTargetDouble>("/left_wheel/target");

  right_wheel_init_client_ = this->create_client<std_srvs::srv::Trigger>("/right_wheel/init");
  right_wheel_mode_client_ = this->create_client<std_srvs::srv::Trigger>("/right_wheel/velocity_mode");
  right_wheel_halt_client_ = this->create_client<std_srvs::srv::Trigger>("/right_wheel/halt");
  right_wheel_recover_client_ = this->create_client<std_srvs::srv::Trigger>("/right_wheel/recover");
  right_wheel_target_client_ = this->create_client<canopen_interfaces::srv::COTargetDouble>("/right_wheel/target");

  const std_msgs::msg::Float64 empty_left_wheel;
  left_wheel_position_ptr_.set(std::make_shared<std_msgs::msg::Float64>(empty_left_wheel));
  left_wheel_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/left_wheel/actual_position", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<std_msgs::msg::Float64> msg) -> void {
        std::shared_ptr<std_msgs::msg::Float64> left_wheel_position;
        left_wheel_position_ptr_.get(left_wheel_position);
        left_wheel_position->data = msg->data * left_wheel_position_transmission_;
      });

  const std_msgs::msg::Float64 empty_right_wheel;
  right_wheel_position_ptr_.set(std::make_shared<std_msgs::msg::Float64>(empty_right_wheel));
  right_wheel_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/right_wheel/actual_position", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<std_msgs::msg::Float64> msg) -> void {
        std::shared_ptr<std_msgs::msg::Float64> right_wheel_position;
        right_wheel_position_ptr_.get(right_wheel_position);
        right_wheel_position->data = msg->data * right_wheel_position_transmission_;
      });

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ = this->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_stamped_subscriber_ = this->create_subscription<Twist>(
        DEFAULT_COMMAND_STAMPED_TOPIC, rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<Twist> msg) -> void {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(this->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }
          if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
          {
            RCLCPP_WARN_ONCE(this->get_logger(),
                             "Received TwistStamped with zero timestamp, setting it to current "
                             "time, this message will only be shown once");
            msg->header.stamp = this->now();
          }
          received_velocity_msg_ptr_.set(std::move(msg));
        });
  }
  else
  {
    velocity_command_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(this->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          // Write fake header in the stored stamped command
          std::shared_ptr<Twist> twist_stamped;
          received_velocity_msg_ptr_.get(twist_stamped);
          twist_stamped->twist = *msg;
          twist_stamped->header.stamp = this->now();
        });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ =
      this->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

  auto& odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  previous_publish_timestamp_ = this->now();

  // initialize odom values zeros
  odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ =
      this->create_publisher<tf2_msgs::msg::TFMessage>(DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto& odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;

  previous_update_timestamp_ = this->now();

  auto left_wheel_init_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto left_wheel_init_result = left_wheel_init_client_->async_send_request(left_wheel_init_request);
  auto right_wheel_init_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto right_wheel_init_result = right_wheel_init_client_->async_send_request(right_wheel_init_request);

  auto left_wheel_mode_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto left_wheel_mode_result = left_wheel_mode_client_->async_send_request(left_wheel_mode_request);
  auto right_wheel_mode_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto right_wheel_mode_result = right_wheel_mode_client_->async_send_request(right_wheel_mode_request);

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveNode::on_activate(const rclcpp_lifecycle::State&)
{
  auto logger = this->get_logger();

  timer_ = this->create_wall_timer(publish_period_.to_chrono<std::chrono::seconds>(),
                                   std::bind(&DiffDriveNode::update, this));

  auto left_wheel_recover_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto left_wheel_recover_result = left_wheel_recover_client_->async_send_request(left_wheel_recover_request);
  auto right_wheel_recover_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto right_wheel_recover_result = right_wheel_recover_client_->async_send_request(right_wheel_recover_request);

  subscriber_is_active_ = true;

  RCLCPP_DEBUG(logger, "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveNode::on_deactivate(const rclcpp_lifecycle::State&)
{
  timer_.reset();

  auto left_wheel_halt_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto left_wheel_halt_result = left_wheel_halt_client_->async_send_request(left_wheel_halt_request);
  auto right_wheel_halt_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto right_wheel_halt_result = right_wheel_halt_client_->async_send_request(right_wheel_halt_request);

  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  left_wheel_position_ptr_.set(std::make_shared<std_msgs::msg::Float64>());
  right_wheel_position_ptr_.set(std::make_shared<std_msgs::msg::Float64>());
  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveNode::on_error(const rclcpp_lifecycle::State&)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveNode::on_shutdown(const rclcpp_lifecycle::State&)
{
  return CallbackReturn::SUCCESS;
}

void DiffDriveNode::update()
{
  auto logger = this->get_logger();

  const auto current_time = this->now();

  std::shared_ptr<Twist> last_msg;
  received_velocity_msg_ptr_.get(last_msg);

  if (last_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
  }

  const auto dt = current_time - last_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (dt > cmd_vel_timeout_)
  {
    last_msg->twist.linear.x = 0.0;
    last_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_msg;
  double& linear_command = command.twist.linear.x;
  double& angular_command = command.twist.angular.z;

  // Apply (possibly new) multipliers:
  const auto wheels = wheel_params_;
  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  if (odom_params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_command, angular_command, current_time);
  }
  else
  {
    std::shared_ptr<std_msgs::msg::Float64> left_position;
    std::shared_ptr<std_msgs::msg::Float64> right_position;
    left_wheel_position_ptr_.get(left_position);
    right_wheel_position_ptr_.get(right_position);
    odometry_.update(left_position->data, right_position->data, current_time);
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  if (previous_publish_timestamp_ + publish_period_ < current_time)
  {
    previous_publish_timestamp_ += publish_period_;

    if (realtime_odometry_publisher_->trylock())
    {
      auto& odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = current_time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinear();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto& transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = current_time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  auto& last_command = previous_commands_.back().twist;
  auto& second_to_last_command = previous_commands_.front().twist;
  limiter_linear_.limit(linear_command, last_command.linear.x, second_to_last_command.linear.x, update_dt.seconds());
  limiter_angular_.limit(angular_command, last_command.angular.z, second_to_last_command.angular.z,
                         update_dt.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(command);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto& limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // Compute wheels velocities:
  const double velocity_left = (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
  const double velocity_right = (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;

  // Set wheels velocities:
  auto left_wheel_target_request = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
  left_wheel_target_request->target = velocity_left / left_wheel_velocity_transmission_;
  auto left_wheel_target_result = left_wheel_target_client_->async_send_request(left_wheel_target_request);
  auto right_wheel_target_request = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
  right_wheel_target_request->target = velocity_right / right_wheel_velocity_transmission_;
  auto right_wheel_target_result = right_wheel_target_client_->async_send_request(right_wheel_target_request);
}

bool DiffDriveNode::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  // left_wheel_init_client_.reset();
  // left_wheel_mode_client_.reset();
  // left_wheel_halt_client_.reset();
  // left_wheel_recover_client_.reset();
  // left_wheel_target_client_.reset();

  // right_wheel_init_client_.reset();
  // right_wheel_mode_client_.reset();
  // right_wheel_halt_client_.reset();
  // right_wheel_recover_client_.reset();
  // right_wheel_target_client_.reset();

  subscriber_is_active_ = false;
  left_wheel_position_subscriber_.reset();
  right_wheel_position_subscriber_.reset();
  velocity_command_stamped_subscriber_.reset();
  velocity_command_subscriber_.reset();

  left_wheel_position_ptr_.set(nullptr);
  right_wheel_position_ptr_.set(nullptr);
  received_velocity_msg_ptr_.set(nullptr);
  return true;
}
}  // namespace diff_drive

int main(int argc, char* argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<diff_drive::DiffDriveNode> diff_drive_node =
      std::make_shared<diff_drive::DiffDriveNode>("diff_drive_node");

  exe.add_node(diff_drive_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
