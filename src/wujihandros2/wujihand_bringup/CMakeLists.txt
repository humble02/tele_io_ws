cmake_minimum_required(VERSION 3.8)
project(wujihand_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wave_demo.py
  scripts/dual_wave_demo.py
  scripts/home.py
  scripts/wujihand_joint_control_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
