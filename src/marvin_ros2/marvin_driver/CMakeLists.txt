cmake_minimum_required(VERSION 3.8)
project(marvin_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(MARVIN_SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/contrlSDK/include)
set(MARVIN_SDK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/vendor/contrlSDK/lib/libMarvinSDK.so)

if(NOT EXISTS ${MARVIN_SDK_LIBRARY})
  message(FATAL_ERROR "Missing Marvin SDK library: ${MARVIN_SDK_LIBRARY}")
endif()

function(configure_marvin_sdk_target target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_definitions(${target} PRIVATE CMPL_LIN)
  target_include_directories(${target} PRIVATE
    ${MARVIN_SDK_INCLUDE_DIR}
  )
  target_link_directories(${target} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/contrlSDK/lib
  )
  target_link_libraries(${target} MarvinSDK)
  set_target_properties(${target} PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/vendor/contrlSDK/lib"
    INSTALL_RPATH "$ORIGIN/.."
  )
endfunction()

add_executable(marvin_driver_node src/marvin_driver_node.cpp)
configure_marvin_sdk_target(marvin_driver_node)
ament_target_dependencies(marvin_driver_node
  rclcpp
  sensor_msgs
  std_srvs
)

add_executable(marvin_link_check_node src/marvin_link_check_node.cpp)
configure_marvin_sdk_target(marvin_link_check_node)
ament_target_dependencies(marvin_link_check_node
  rclcpp
)

add_executable(marvin_cmd_delay_check_node src/marvin_cmd_delay_check_node.cpp)
configure_marvin_sdk_target(marvin_cmd_delay_check_node)
ament_target_dependencies(marvin_cmd_delay_check_node
  rclcpp
)

install(TARGETS
  marvin_driver_node
  marvin_link_check_node
  marvin_cmd_delay_check_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${MARVIN_SDK_LIBRARY}
  DESTINATION lib
)

install(DIRECTORY vendor/contrlSDK/include/
  DESTINATION include/${PROJECT_NAME}/vendor/contrlSDK
)

ament_package()
