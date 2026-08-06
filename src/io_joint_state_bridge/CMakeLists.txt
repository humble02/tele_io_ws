cmake_minimum_required(VERSION 3.8)
project(io_joint_state_bridge)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/io_joint_state_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME io_joint_state_bridge
)

install(
  PROGRAMS scripts/footswitch_pedal.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME footswitch_pedal
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY udev DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_freeze_gate test/test_freeze_gate.py)
endif()

ament_package()
