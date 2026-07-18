cmake_minimum_required(VERSION 3.8)
project(science_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_library(PHIDGET22_LIB phidget22 REQUIRED)

add_library(science_2026 SHARED
  src/science_2026_system.cpp
)

add_executable(science_2026_keyboard_teleop
  teleop/science_2026_keyboard_teleop.cpp
)

target_include_directories(science_2026 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(science_2026
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
)

ament_target_dependencies(science_2026_keyboard_teleop
  rclcpp
  sensor_msgs
  std_msgs
)


target_link_libraries(science_2026
  ${PHIDGET22_LIB}
)

target_compile_features(science_2026 PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(
  hardware_interface
  plugin.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    science_2026
    science_2026_keyboard_teleop
    
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
  std_msgs
)

ament_package()
