cmake_minimum_required(VERSION 3.8)
project(marvin_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(marvin_zero_position_node src/marvin_zero_position_node.cpp)
target_compile_features(marvin_zero_position_node PUBLIC cxx_std_17)
ament_target_dependencies(marvin_zero_position_node
  rclcpp
  sensor_msgs
)

install(TARGETS marvin_zero_position_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/marvin_joint_control_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
