#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "MarvinSDK.h"

namespace
{
constexpr std::size_t kJointCount = 7;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr auto kSdkCallSpacing = std::chrono::milliseconds(2);
constexpr auto kModeStepSpacing = std::chrono::milliseconds(200);
constexpr long kModeResponseTimeoutMs = 1000;
constexpr int kPositionState = 1;
constexpr int kTorqueState = 3;
constexpr int kJointImpedanceType = 1;

std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::vector<std::string> default_joint_names(const std::string & suffix)
{
  std::vector<std::string> names;
  names.reserve(kJointCount);
  for (std::size_t i = 1; i <= kJointCount; ++i) {
    names.emplace_back("Joint" + std::to_string(i) + "_" + suffix);
  }
  return names;
}

bool parse_ip_address(const std::string & text, std::array<unsigned char, 4> & ip)
{
  std::stringstream stream(text);
  std::string part;
  std::size_t idx = 0;
  while (std::getline(stream, part, '.')) {
    if (idx >= ip.size() || part.empty()) {
      return false;
    }
    int value = -1;
    try {
      std::size_t consumed = 0;
      value = std::stoi(part, &consumed);
      if (consumed != part.size()) {
        return false;
      }
    } catch (...) {
      return false;
    }
    if (value < 0 || value > 255) {
      return false;
    }
    ip[idx++] = static_cast<unsigned char>(value);
  }
  return idx == ip.size();
}

std::array<double, kJointCount> get_double_array(
  rclcpp::Node & node, const std::string & name,
  const std::array<double, kJointCount> & fallback)
{
  const auto values = node.get_parameter(name).as_double_array();
  if (values.size() != kJointCount) {
    RCLCPP_WARN(
      node.get_logger(), "Parameter '%s' must contain 7 values; using fallback.", name.c_str());
    return fallback;
  }

  std::array<double, kJointCount> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

}  // namespace

class MarvinDriverNode : public rclcpp::Node
{
public:
  MarvinDriverNode()
  : Node("marvin_driver")
  {
    declare_parameters();
    read_parameters();
    create_ros_interfaces();

    if (auto_connect_) {
      connect_robot();
    }
  }

  ~MarvinDriverNode() override
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (connected_) {
      OnRelease();
      connected_ = false;
    }
  }

private:
  struct ArmRuntime
  {
    std::string side;
    char sdk_arm{'A'};
    int sdk_index{0};
    bool enabled{false};
    bool mode_ready{false};
    bool has_command{false};
    bool timeout_reported{false};
    std::vector<std::string> joint_names;
    std::array<double, kJointCount> last_feedback_rad{};
    rclcpp::Time last_command_time;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv;
  };

  void declare_parameters()
  {
    this->declare_parameter<std::string>("robot_ip", "192.168.1.190");
    this->declare_parameter<int>("log_switch", 0);
    this->declare_parameter<bool>("auto_connect", true);
    this->declare_parameter<std::string>("arms", "both");
    this->declare_parameter<std::string>("control_mode", "position");
    this->declare_parameter<int>("velocity_ratio", 10);
    this->declare_parameter<int>("acceleration_ratio", 10);
    this->declare_parameter<double>("feedback_rate_hz", 50.0);
    this->declare_parameter<double>("watchdog_rate_hz", 5.0);
    this->declare_parameter<double>("command_timeout_sec", 0.5);
    this->declare_parameter<bool>("disable_on_timeout", false);
    this->declare_parameter<std::vector<std::string>>("left_joint_names", default_joint_names("L"));
    this->declare_parameter<std::vector<std::string>>("right_joint_names", default_joint_names("R"));
    this->declare_parameter<std::vector<double>>(
      "joint_impedance_k", std::vector<double>{3.0, 3.0, 3.0, 1.6, 1.0, 1.0, 1.0});
    this->declare_parameter<std::vector<double>>(
      "joint_impedance_d", std::vector<double>{0.6, 0.6, 0.6, 0.4, 0.2, 0.2, 0.2});
  }

  void read_parameters()
  {
    robot_ip_ = this->get_parameter("robot_ip").as_string();
    log_switch_ = this->get_parameter("log_switch").as_int();
    auto_connect_ = this->get_parameter("auto_connect").as_bool();
    control_mode_ = to_lower(this->get_parameter("control_mode").as_string());
    velocity_ratio_ = static_cast<int>(this->get_parameter("velocity_ratio").as_int());
    acceleration_ratio_ = static_cast<int>(this->get_parameter("acceleration_ratio").as_int());
    feedback_rate_hz_ = this->get_parameter("feedback_rate_hz").as_double();
    watchdog_rate_hz_ = this->get_parameter("watchdog_rate_hz").as_double();
    command_timeout_sec_ = this->get_parameter("command_timeout_sec").as_double();
    disable_on_timeout_ = this->get_parameter("disable_on_timeout").as_bool();

    const std::array<double, kJointCount> default_k{3.0, 3.0, 3.0, 1.6, 1.0, 1.0, 1.0};
    const std::array<double, kJointCount> default_d{0.6, 0.6, 0.6, 0.4, 0.2, 0.2, 0.2};
    joint_impedance_k_ = get_double_array(*this, "joint_impedance_k", default_k);
    joint_impedance_d_ = get_double_array(*this, "joint_impedance_d", default_d);

    arms_[0].side = "left";
    arms_[0].sdk_arm = 'A';
    arms_[0].sdk_index = 0;
    arms_[0].joint_names = this->get_parameter("left_joint_names").as_string_array();
    arms_[1].side = "right";
    arms_[1].sdk_arm = 'B';
    arms_[1].sdk_index = 1;
    arms_[1].joint_names = this->get_parameter("right_joint_names").as_string_array();

    for (auto & arm : arms_) {
      if (arm.joint_names.size() != kJointCount) {
        RCLCPP_WARN(
          this->get_logger(), "%s_joint_names must contain 7 names; using URDF defaults.",
          arm.side.c_str());
        arm.joint_names = default_joint_names(arm.sdk_arm == 'A' ? "L" : "R");
      }
      arm.last_command_time = this->now();
    }

    const auto arms_param = to_lower(this->get_parameter("arms").as_string());
    if (arms_param == "both" || arms_param == "all" || arms_param == "ab") {
      arms_[0].enabled = true;
      arms_[1].enabled = true;
    } else if (arms_param == "left" || arms_param == "a") {
      arms_[0].enabled = true;
    } else if (arms_param == "right" || arms_param == "b") {
      arms_[1].enabled = true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown arms='%s'; using both arms.", arms_param.c_str());
      arms_[0].enabled = true;
      arms_[1].enabled = true;
    }

    if (control_mode_ != "position" && control_mode_ != "joint_impedance" &&
      control_mode_ != "impedance" && control_mode_ != "joint_impedance_pd" &&
      control_mode_ != "impedance_pd")
    {
      RCLCPP_WARN(
        this->get_logger(), "Unknown control_mode='%s'; using position mode.",
        control_mode_.c_str());
      control_mode_ = "position";
    }
  }

  void create_ros_interfaces()
  {
    for (auto & arm_ref : arms_) {
      auto * arm = &arm_ref;
      if (!arm->enabled) {
        continue;
      }

      arm->state_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        arm->side + "/joint_states", rclcpp::SensorDataQoS());
      arm->command_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        arm->side + "/joint_commands", rclcpp::SensorDataQoS(),
        [this, arm](sensor_msgs::msg::JointState::SharedPtr msg) {
          this->handle_joint_command(*arm, *msg);
        });
      arm->disable_srv = this->create_service<std_srvs::srv::Trigger>(
        arm->side + "/disable",
        [this, arm](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          this->handle_disable(*arm, response);
        });
    }

    connect_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "connect",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = this->connect_robot();
        response->message = response->success ? "connected" : "connect failed";
      });

    release_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "release",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->release_robot();
        response->success = true;
        response->message = "released";
      });

    estop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "estop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        if (!connected_) {
          response->success = false;
          response->message = "not connected";
          return;
        }
        EStop("AB");
        sdk_call_spacing();
        response->success = true;
        response->message = "estop sent to both arms";
      });

    feedback_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(feedback_rate_hz_, 1.0)),
      std::bind(&MarvinDriverNode::publish_feedback, this));

    watchdog_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(watchdog_rate_hz_, 1.0)),
      std::bind(&MarvinDriverNode::watchdog_tick, this));
  }

  bool connect_robot()
  {
    std::array<unsigned char, 4> ip{};
    if (!parse_ip_address(robot_ip_, ip)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid robot_ip: '%s'", robot_ip_.c_str());
      return false;
    }

    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (connected_) {
      const bool ready = configure_enabled_arms();
      if (!ready) {
        RCLCPP_WARN(this->get_logger(), "Marvin controller is connected, but not all arms are ready.");
      }
      return ready;
    }

    if (!Connect(ip[0], ip[1], ip[2], ip[3], log_switch_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect Marvin controller at %s", robot_ip_.c_str());
      return false;
    }
    sdk_call_spacing();
    connected_ = true;
    reset_command_state();

    const bool all_modes_ready = configure_enabled_arms();
    if (all_modes_ready) {
      RCLCPP_INFO(
        this->get_logger(), "Connected Marvin controller at %s in %s mode.",
        robot_ip_.c_str(), control_mode_.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Connected Marvin controller at %s, but mode setup is incomplete.",
        robot_ip_.c_str());
    }
    return all_modes_ready;
  }

  bool configure_enabled_arms()
  {
    bool all_modes_ready = true;
    for (auto & arm : arms_) {
      if (!arm.enabled) {
        continue;
      }
      if (arm.mode_ready) {
        continue;
      }
      arm.mode_ready = configure_arm_mode(arm);
      all_modes_ready = all_modes_ready && arm.mode_ready;
      if (arm.mode_ready) {
        reset_command_state(arm);
      }
      sdk_call_spacing();
    }
    return all_modes_ready;
  }

  bool configure_arm_mode(ArmRuntime & arm)
  {
    if (control_mode_ == "position") {
      if (!configure_position_mode(arm)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to set %s arm to position mode with velocity_ratio=%d acceleration_ratio=%d.",
          arm.side.c_str(), velocity_ratio_, acceleration_ratio_);
        return false;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Configured %s arm position mode with velocity_ratio=%d acceleration_ratio=%d.",
        arm.side.c_str(), velocity_ratio_, acceleration_ratio_);
      sdk_call_spacing();
      return true;
    }

    const bool pd_feedforward =
      control_mode_ == "joint_impedance_pd" || control_mode_ == "impedance_pd";
    if (!set_joint_pid_control_type(arm, pd_feedforward ? 1 : 0)) {
      return false;
    }

    if (!configure_joint_impedance_mode(arm)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to set %s arm to joint impedance mode with velocity_ratio=%d acceleration_ratio=%d.",
        arm.side.c_str(), velocity_ratio_, acceleration_ratio_);
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Configured %s arm joint impedance mode with velocity_ratio=%d acceleration_ratio=%d.",
      arm.side.c_str(), velocity_ratio_, acceleration_ratio_);
    sdk_call_spacing();
    return true;
  }

  bool configure_position_mode(const ArmRuntime & arm)
  {
    if (!OnClearSet()) {
      RCLCPP_ERROR(this->get_logger(), "OnClearSet failed before setting %s arm velocity/acceleration.", arm.side.c_str());
      return false;
    }
    if (!queue_joint_limit(arm)) {
      return false;
    }
    if (!send_queued_sdk_set("joint velocity/acceleration limit", arm)) {
      return false;
    }

    std::this_thread::sleep_for(kModeStepSpacing);

    if (!OnClearSet()) {
      RCLCPP_ERROR(this->get_logger(), "OnClearSet failed before setting %s arm state.", arm.side.c_str());
      return false;
    }
    if (!queue_target_state(arm, kPositionState)) {
      return false;
    }
    if (!send_queued_sdk_set("position mode state", arm)) {
      return false;
    }
    log_controller_joint_limit(arm);
    return true;
  }

  bool configure_joint_impedance_mode(const ArmRuntime & arm)
  {
    if (!OnClearSet()) {
      RCLCPP_ERROR(this->get_logger(), "OnClearSet failed before setting %s arm impedance mode.", arm.side.c_str());
      return false;
    }
    if (!queue_joint_limit(arm)) {
      return false;
    }
    if (!queue_joint_kd(arm)) {
      return false;
    }
    if (!queue_target_state(arm, kTorqueState)) {
      return false;
    }
    if (!queue_impedance_type(arm, kJointImpedanceType)) {
      return false;
    }
    if (!send_queued_sdk_set("joint impedance mode", arm)) {
      return false;
    }
    log_controller_joint_limit(arm);
    return true;
  }

  bool queue_joint_limit(const ArmRuntime & arm)
  {
    if (arm.sdk_arm == 'A') {
      return OnSetJointLmt_A(velocity_ratio_, acceleration_ratio_);
    }
    return OnSetJointLmt_B(velocity_ratio_, acceleration_ratio_);
  }

  bool queue_target_state(const ArmRuntime & arm, const int state)
  {
    if (arm.sdk_arm == 'A') {
      return OnSetTargetState_A(state);
    }
    return OnSetTargetState_B(state);
  }

  bool queue_joint_kd(const ArmRuntime & arm)
  {
    if (arm.sdk_arm == 'A') {
      return OnSetJointKD_A(joint_impedance_k_.data(), joint_impedance_d_.data());
    }
    return OnSetJointKD_B(joint_impedance_k_.data(), joint_impedance_d_.data());
  }

  bool queue_impedance_type(const ArmRuntime & arm, const int type)
  {
    if (arm.sdk_arm == 'A') {
      return OnSetImpType_A(type);
    }
    return OnSetImpType_B(type);
  }

  bool send_queued_sdk_set(const char * action, const ArmRuntime & arm)
  {
    const auto delay = OnSetSendWaitResponse(kModeResponseTimeoutMs);
    sdk_call_spacing();
    if (delay < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "Timed out sending %s for %s arm after %ld ms.",
        action, arm.side.c_str(), kModeResponseTimeoutMs);
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "Sent %s for %s arm; response delay=%ld ms.",
      action, arm.side.c_str(), delay);
    return true;
  }

  void log_controller_joint_limit(const ArmRuntime & arm)
  {
    DCSS dcss{};
    if (!OnGetBuf(&dcss)) {
      RCLCPP_WARN(this->get_logger(), "Could not read controller joint limits for %s arm.", arm.side.c_str());
      return;
    }

    const auto & input = dcss.m_In[arm.sdk_index];
    RCLCPP_INFO(
      this->get_logger(),
      "Controller reports %s arm velocity_ratio=%d acceleration_ratio=%d state=%d.",
      arm.side.c_str(), static_cast<int>(input.m_Joint_Vel_Ratio),
      static_cast<int>(input.m_Joint_Acc_Ratio), dcss.m_State[arm.sdk_index].m_CurState);
  }

  bool set_joint_pid_control_type(const ArmRuntime & arm, const long value)
  {
    char parameter_name[30]{};
    const char * name = arm.sdk_arm == 'A' ?
      "R.A0.BASIC.JointPIDCtlType" : "R.A1.BASIC.JointPIDCtlType";
    std::snprintf(parameter_name, sizeof(parameter_name), "%s", name);

    const auto result = OnSetIntPara(parameter_name, value);
    if (result < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to set %s to %ld for %s arm.",
        name, value, arm.side.c_str());
      return false;
    }
    sdk_call_spacing();
    return true;
  }

  void release_robot()
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (connected_) {
      OnRelease();
      sdk_call_spacing();
      connected_ = false;
    }
    for (auto & arm : arms_) {
      arm.mode_ready = false;
      reset_command_state(arm);
    }
  }

  void reset_command_state()
  {
    for (auto & arm : arms_) {
      reset_command_state(arm);
    }
  }

  void reset_command_state(ArmRuntime & arm)
  {
    arm.has_command = false;
    arm.timeout_reported = false;
    arm.last_command_time = this->now();
  }

  void sdk_call_spacing()
  {
    std::this_thread::sleep_for(kSdkCallSpacing);
  }

  void handle_disable(
    ArmRuntime & arm, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (!connected_) {
      response->success = false;
      response->message = "not connected";
      return;
    }
    response->success = Disable(arm.sdk_arm);
    sdk_call_spacing();
    if (response->success) {
      arm.mode_ready = false;
      reset_command_state(arm);
    }
    response->message = response->success ? "disabled" : "disable failed";
  }

  bool command_to_array(
    const ArmRuntime & arm, const sensor_msgs::msg::JointState & msg,
    std::array<double, kJointCount> & command_rad)
  {
    if (msg.position.size() < kJointCount) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "%s/joint_commands requires at least 7 positions.", arm.side.c_str());
      return false;
    }

    if (msg.name.empty()) {
      std::copy_n(msg.position.begin(), kJointCount, command_rad.begin());
    } else {
      std::map<std::string, double> by_name;
      const auto count = std::min(msg.name.size(), msg.position.size());
      for (std::size_t i = 0; i < count; ++i) {
        by_name[msg.name[i]] = msg.position[i];
      }
      for (std::size_t i = 0; i < kJointCount; ++i) {
        const auto iter = by_name.find(arm.joint_names[i]);
        if (iter == by_name.end()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "%s/joint_commands missing joint '%s'.", arm.side.c_str(), arm.joint_names[i].c_str());
          return false;
        }
        command_rad[i] = iter->second;
      }
    }

    for (const auto value : command_rad) {
      if (!std::isfinite(value)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "%s/joint_commands contains NaN or Inf.", arm.side.c_str());
        return false;
      }
    }
    return true;
  }

  void handle_joint_command(ArmRuntime & arm, const sensor_msgs::msg::JointState & msg)
  {
    if (!arm.enabled) {
      return;
    }
    if (!connected_ || !arm.mode_ready) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Ignoring %s arm command because driver is not ready.", arm.side.c_str());
      return;
    }

    std::array<double, kJointCount> command_rad{};
    if (!command_to_array(arm, msg, command_rad)) {
      return;
    }

    double command_deg[kJointCount]{};
    for (std::size_t i = 0; i < kJointCount; ++i) {
      command_deg[i] = command_rad[i] * kRadToDeg;
    }

    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      if (!SetJointPostionCmd(arm.sdk_arm, command_deg)) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Failed to send %s arm joint command.", arm.side.c_str());
        return;
      }
    }

    arm.last_command_time = this->now();
    arm.has_command = true;
    arm.timeout_reported = false;
  }

  void publish_feedback()
  {
    if (!connected_) {
      return;
    }

    DCSS dcss{};
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      if (!OnGetBuf(&dcss)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000, "OnGetBuf failed.");
        return;
      }
    }

    const auto stamp = this->now();
    for (auto & arm : arms_) {
      if (!arm.enabled || !arm.state_pub) {
        continue;
      }

      const auto & out = dcss.m_Out[arm.sdk_index];
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = stamp;
      msg.name = arm.joint_names;
      msg.position.resize(kJointCount);
      msg.velocity.resize(kJointCount);
      msg.effort.resize(kJointCount);

      for (std::size_t i = 0; i < kJointCount; ++i) {
        msg.position[i] = static_cast<double>(out.m_FB_Joint_Pos[i]) * kDegToRad;
        msg.velocity[i] = static_cast<double>(out.m_FB_Joint_Vel[i]) * kDegToRad;
        msg.effort[i] = static_cast<double>(out.m_FB_Joint_SToq[i]);
        arm.last_feedback_rad[i] = msg.position[i];
      }

      arm.state_pub->publish(msg);
    }
  }

  void watchdog_tick()
  {
    if (!connected_ || command_timeout_sec_ <= 0.0) {
      return;
    }

    const auto now = this->now();
    for (auto & arm : arms_) {
      if (!arm.enabled || !arm.has_command || !arm.mode_ready) {
        continue;
      }

      const auto age = (now - arm.last_command_time).seconds();
      if (age <= command_timeout_sec_) {
        continue;
      }

      if (!arm.timeout_reported) {
        RCLCPP_WARN(
          this->get_logger(), "%s arm command timeout after %.3f s.", arm.side.c_str(), age);
        arm.timeout_reported = true;
      }

      if (disable_on_timeout_) {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        const bool disabled = Disable(arm.sdk_arm);
        sdk_call_spacing();
        if (disabled) {
          arm.mode_ready = false;
          reset_command_state(arm);
          RCLCPP_WARN(this->get_logger(), "%s arm disabled by watchdog.", arm.side.c_str());
        }
      }
    }
  }

  std::string robot_ip_;
  std::string control_mode_;
  int log_switch_{0};
  bool auto_connect_{true};
  int velocity_ratio_{10};
  int acceleration_ratio_{10};
  double feedback_rate_hz_{50.0};
  double watchdog_rate_hz_{5.0};
  double command_timeout_sec_{0.5};
  bool disable_on_timeout_{false};
  bool connected_{false};
  std::array<double, kJointCount> joint_impedance_k_{};
  std::array<double, kJointCount> joint_impedance_d_{};
  std::array<ArmRuntime, 2> arms_;
  std::mutex sdk_mutex_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarvinDriverNode>());
  rclcpp::shutdown();
  return 0;
}
