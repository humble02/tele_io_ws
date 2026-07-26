#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
constexpr std::size_t kJointCount = 7;

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

std::array<double, kJointCount> zero_array()
{
  std::array<double, kJointCount> values{};
  values.fill(0.0);
  return values;
}

std::array<double, kJointCount> parameter_array(
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

class MarvinZeroPositionNode : public rclcpp::Node
{
public:
  MarvinZeroPositionNode()
  : Node("marvin_zero_position")
  {
    declare_parameters();
    read_parameters();
    create_interfaces();

    start_time_ = this->now();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(command_rate_hz_, 1.0)),
      std::bind(&MarvinZeroPositionNode::tick, this));
  }

private:
  struct Arm
  {
    std::string side;
    std::vector<std::string> joint_names;
    std::array<double, kJointCount> current{};
    std::array<double, kJointCount> start{};
    std::array<double, kJointCount> target{};
    bool enabled{false};
    bool feedback_received{false};
    bool started{false};
    bool done{false};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub;
  };

  void declare_parameters()
  {
    this->declare_parameter<std::string>("arms", "both");
    this->declare_parameter<double>("command_rate_hz", 50.0);
    this->declare_parameter<double>("hold_before_move_sec", 0.5);
    this->declare_parameter<double>("move_duration_sec", 5.0);
    this->declare_parameter<double>("timeout_sec", 30.0);
    this->declare_parameter<double>("tolerance_rad", 0.02);
    this->declare_parameter<bool>("exit_on_success", true);
    this->declare_parameter<std::vector<std::string>>("left_joint_names", default_joint_names("L"));
    this->declare_parameter<std::vector<std::string>>("right_joint_names", default_joint_names("R"));
    this->declare_parameter<std::vector<double>>("left_target", std::vector<double>(kJointCount, 0.0));
    this->declare_parameter<std::vector<double>>("right_target", std::vector<double>(kJointCount, 0.0));
  }

  void read_parameters()
  {
    command_rate_hz_ = this->get_parameter("command_rate_hz").as_double();
    hold_before_move_sec_ = this->get_parameter("hold_before_move_sec").as_double();
    move_duration_sec_ = this->get_parameter("move_duration_sec").as_double();
    timeout_sec_ = this->get_parameter("timeout_sec").as_double();
    tolerance_rad_ = this->get_parameter("tolerance_rad").as_double();
    exit_on_success_ = this->get_parameter("exit_on_success").as_bool();

    arms_[0].side = "left";
    arms_[0].joint_names = this->get_parameter("left_joint_names").as_string_array();
    arms_[0].target = parameter_array(*this, "left_target", zero_array());
    arms_[1].side = "right";
    arms_[1].joint_names = this->get_parameter("right_joint_names").as_string_array();
    arms_[1].target = parameter_array(*this, "right_target", zero_array());

    for (std::size_t i = 0; i < arms_.size(); ++i) {
      auto & arm = arms_[i];
      if (arm.joint_names.size() != kJointCount) {
        arm.joint_names = default_joint_names(i == 0 ? "L" : "R");
      }
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
  }

  void create_interfaces()
  {
    for (auto & arm_ref : arms_) {
      auto * arm = &arm_ref;
      if (!arm->enabled) {
        continue;
      }

      arm->command_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        arm->side + "/joint_commands", rclcpp::SensorDataQoS());
      arm->state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        arm->side + "/joint_states", rclcpp::SensorDataQoS(),
        [this, arm](sensor_msgs::msg::JointState::SharedPtr msg) {
          this->handle_feedback(*arm, *msg);
        });
    }
  }

  void handle_feedback(Arm & arm, const sensor_msgs::msg::JointState & msg)
  {
    std::array<double, kJointCount> positions{};

    if (msg.name.empty()) {
      if (msg.position.size() < kJointCount) {
        return;
      }
      std::copy_n(msg.position.begin(), kJointCount, positions.begin());
    } else {
      if (msg.position.size() < msg.name.size()) {
        return;
      }
      std::map<std::string, double> by_name;
      for (std::size_t i = 0; i < msg.name.size(); ++i) {
        by_name[msg.name[i]] = msg.position[i];
      }
      for (std::size_t i = 0; i < kJointCount; ++i) {
        const auto iter = by_name.find(arm.joint_names[i]);
        if (iter == by_name.end()) {
          return;
        }
        positions[i] = iter->second;
      }
    }

    for (const auto value : positions) {
      if (!std::isfinite(value)) {
        return;
      }
    }

    arm.current = positions;
    if (!arm.feedback_received) {
      arm.feedback_received = true;
      arm.start = arm.current;
      RCLCPP_INFO(this->get_logger(), "Received initial %s arm feedback.", arm.side.c_str());
    }
  }

  void tick()
  {
    const auto now = this->now();
    if (!all_feedback_ready()) {
      if (!timeout_reported_ && timeout_sec_ > 0.0 && (now - start_time_).seconds() > timeout_sec_) {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for initial Marvin joint feedback.");
        timeout_reported_ = true;
        if (exit_on_success_) {
          rclcpp::shutdown();
        }
      }
      return;
    }

    if (!motion_started_) {
      motion_started_ = true;
      motion_start_time_ = now;
      for (auto & arm : arms_) {
        if (arm.enabled) {
          arm.start = arm.current;
          arm.started = true;
        }
      }
      RCLCPP_INFO(this->get_logger(), "Initial feedback captured; starting zero-position command.");
    }

    const auto elapsed = (now - motion_start_time_).seconds();
    const auto move_elapsed = std::max(0.0, elapsed - hold_before_move_sec_);
    const auto alpha = move_duration_sec_ <= 0.0 ? 1.0 : std::clamp(move_elapsed / move_duration_sec_, 0.0, 1.0);

    bool all_done = true;
    for (auto & arm : arms_) {
      if (!arm.enabled || !arm.command_pub) {
        continue;
      }
      publish_command(arm, alpha, now);
      arm.done = alpha >= 1.0 && reached_target(arm);
      all_done = all_done && arm.done;
    }

    if (all_done && !success_reported_) {
      RCLCPP_INFO(this->get_logger(), "Marvin zero-position command completed.");
      success_reported_ = true;
      if (exit_on_success_) {
        rclcpp::shutdown();
      }
    }
  }

  bool all_feedback_ready() const
  {
    for (const auto & arm : arms_) {
      if (arm.enabled && !arm.feedback_received) {
        return false;
      }
    }
    return true;
  }

  void publish_command(const Arm & arm, const double alpha, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = arm.joint_names;
    msg.position.resize(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      msg.position[i] = arm.start[i] + (arm.target[i] - arm.start[i]) * alpha;
    }
    arm.command_pub->publish(msg);
  }

  bool reached_target(const Arm & arm) const
  {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (std::abs(arm.current[i] - arm.target[i]) > tolerance_rad_) {
        return false;
      }
    }
    return true;
  }

  std::array<Arm, 2> arms_;
  double command_rate_hz_{50.0};
  double hold_before_move_sec_{0.5};
  double move_duration_sec_{5.0};
  double timeout_sec_{30.0};
  double tolerance_rad_{0.02};
  bool exit_on_success_{true};
  bool motion_started_{false};
  bool timeout_reported_{false};
  bool success_reported_{false};
  rclcpp::Time start_time_;
  rclcpp::Time motion_start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarvinZeroPositionNode>());
  rclcpp::shutdown();
  return 0;
}
