#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "MarvinSDK.h"

namespace
{
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

std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool set_joint_limit(const char arm, const int ratio)
{
  OnClearSet();
  if (arm == 'A') {
    OnSetJointLmt_A(ratio, ratio);
  } else {
    OnSetJointLmt_B(ratio, ratio);
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("marvin_cmd_delay_check");

  node->declare_parameter<std::string>("robot_ip", "192.168.1.190");
  node->declare_parameter<std::string>("arm", "A");
  node->declare_parameter<int>("wait_response_ms", 100);
  node->declare_parameter<std::vector<int64_t>>("ratios", std::vector<int64_t>{10, 20, 30});

  const auto robot_ip = node->get_parameter("robot_ip").as_string();
  const auto arm_text = to_lower(node->get_parameter("arm").as_string());
  const auto wait_response_ms =
    std::max(1, static_cast<int>(node->get_parameter("wait_response_ms").as_int()));
  const auto ratios = node->get_parameter("ratios").as_integer_array();

  char arm = 'A';
  if (arm_text == "b" || arm_text == "right") {
    arm = 'B';
  } else if (arm_text != "a" && arm_text != "left") {
    RCLCPP_WARN(node->get_logger(), "Unknown arm='%s'; using A.", arm_text.c_str());
  }

  std::array<unsigned char, 4> ip{};
  if (!parse_ip_address(robot_ip, ip)) {
    RCLCPP_ERROR(node->get_logger(), "Invalid robot_ip: '%s'", robot_ip.c_str());
    rclcpp::shutdown();
    return 2;
  }

  if (!OnLinkTo(ip[0], ip[1], ip[2], ip[3])) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect Marvin controller at %s.", robot_ip.c_str());
    rclcpp::shutdown();
    return 3;
  }

  bool ok = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  for (const auto raw_ratio : ratios) {
    const auto ratio = static_cast<int>(std::clamp<int64_t>(raw_ratio, 1, 100));
    set_joint_limit(arm, ratio);
    const auto delay = OnSetSendWaitResponse(wait_response_ms);
    if (delay < 0) {
      RCLCPP_ERROR(
        node->get_logger(), "Arm %c joint limit ratio=%d timed out in %d ms.",
        arm, ratio, wait_response_ms);
      ok = false;
    } else {
      RCLCPP_INFO(
        node->get_logger(), "Arm %c joint limit ratio=%d response delay=%ld ms.",
        arm, ratio, delay);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  OnRelease();
  RCLCPP_INFO(node->get_logger(), "Marvin command delay check %s.", ok ? "passed" : "failed");
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
