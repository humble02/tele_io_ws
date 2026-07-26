#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>
#include <thread>

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

void clear_arm_error(const char arm)
{
  OnClearSet();
  if (arm == 'A') {
    OnClearErr_A();
  } else {
    OnClearErr_B();
  }
  OnSetSend();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("marvin_link_check");

  node->declare_parameter<std::string>("robot_ip", "192.168.1.190");
  node->declare_parameter<bool>("clear_errors", false);
  node->declare_parameter<int>("frame_samples", 5);
  node->declare_parameter<int>("sample_period_ms", 10);

  const auto robot_ip = node->get_parameter("robot_ip").as_string();
  const auto clear_errors = node->get_parameter("clear_errors").as_bool();
  const auto frame_samples = std::max(1, static_cast<int>(node->get_parameter("frame_samples").as_int()));
  const auto sample_period_ms =
    std::max(1, static_cast<int>(node->get_parameter("sample_period_ms").as_int()));

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

  DCSS dcss{};
  OnGetBuf(&dcss);
  for (int idx = 0; idx < 2; ++idx) {
    const char arm = idx == 0 ? 'A' : 'B';
    const auto arm_error = dcss.m_State[idx].m_ERRCode;
    const auto arm_state = dcss.m_State[idx].m_CurState;
    RCLCPP_INFO(
      node->get_logger(), "Arm %c state=%d error=%d.", arm, arm_state, arm_error);
    if (arm_error != 0 || arm_state == 100) {
      if (clear_errors) {
        RCLCPP_WARN(node->get_logger(), "Clearing arm %c error.", arm);
        clear_arm_error(arm);
      } else {
        ok = false;
      }
    }
  }

  long err_a[7]{};
  long err_b[7]{};
  OnGetServoErr_A(err_a);
  OnGetServoErr_B(err_b);
  bool servo_a_ok = true;
  bool servo_b_ok = true;
  for (int i = 0; i < 7; ++i) {
    if (err_a[i] != 0) {
      RCLCPP_WARN(node->get_logger(), "Arm A servo %d error=%ld.", i, err_a[i]);
      servo_a_ok = false;
    }
    if (err_b[i] != 0) {
      RCLCPP_WARN(node->get_logger(), "Arm B servo %d error=%ld.", i, err_b[i]);
      servo_b_ok = false;
    }
  }
  if (clear_errors && !servo_a_ok) {
    RCLCPP_WARN(node->get_logger(), "Clearing arm A servo errors.");
    clear_arm_error('A');
  }
  if (clear_errors && !servo_b_ok) {
    RCLCPP_WARN(node->get_logger(), "Clearing arm B servo errors.");
    clear_arm_error('B');
  }
  if (clear_errors && (!servo_a_ok || !servo_b_ok)) {
    servo_a_ok = true;
    servo_b_ok = true;
    OnGetServoErr_A(err_a);
    OnGetServoErr_B(err_b);
    for (int i = 0; i < 7; ++i) {
      if (err_a[i] != 0) {
        RCLCPP_WARN(node->get_logger(), "Arm A servo %d error still present=%ld.", i, err_a[i]);
        servo_a_ok = false;
      }
      if (err_b[i] != 0) {
        RCLCPP_WARN(node->get_logger(), "Arm B servo %d error still present=%ld.", i, err_b[i]);
        servo_b_ok = false;
      }
    }
  }
  ok = ok && servo_a_ok && servo_b_ok;

  int changed = 0;
  int previous_frame = 0;
  for (int i = 0; i < frame_samples; ++i) {
    OnGetBuf(&dcss);
    const auto frame = dcss.m_Out[0].m_OutFrameSerial;
    RCLCPP_INFO(node->get_logger(), "Arm A output frame=%d.", frame);
    if (frame != 0 && frame != previous_frame) {
      ++changed;
      previous_frame = frame;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sample_period_ms));
  }

  if (changed == 0) {
    RCLCPP_ERROR(node->get_logger(), "Output frame did not update; check UDP/firewall/link state.");
    ok = false;
  }

  OnRelease();
  RCLCPP_INFO(node->get_logger(), "Marvin link check %s.", ok ? "passed" : "failed");
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
