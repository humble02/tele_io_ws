cmake_minimum_required(VERSION 3.8)
project(io_joint_state_bridge)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/io_joint_state_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME io_joint_state_bridge
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

