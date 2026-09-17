cmake_minimum_required(VERSION 3.16)
project(rapidcode_system)

# rsi.h requires C++20.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rapidcode_trajectory_transfer REQUIRED)

# RapidCode is provided by the installed 'rmp' Debian package (supplied by RSI),
# NOT by rosdep. /rsi/cmake/RSI.cmake defines the imported target RSI::RapidCode
# (IMPORTED_LOCATION /rsi/librapidcode.so) and RMP_INSTALL_PATH; the public
# headers (rsi.h, ...) live in /usr/include, so no extra include dir is needed.
set(RSI_CMAKE "/rsi/cmake/RSI.cmake")
if(NOT EXISTS "${RSI_CMAKE}")
  message(FATAL_ERROR
    "Could not find ${RSI_CMAKE}. Install the rmp Debian package "
    "(supplied by RSI) before building rapidcode_system.")
endif()
include(${RSI_CMAKE})

add_library(rapidcode_system SHARED src/rapidcode_system_hardware.cpp)
target_compile_features(rapidcode_system PUBLIC cxx_std_20)
target_include_directories(rapidcode_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rapidcode_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rapidcode_trajectory_transfer)
target_link_libraries(rapidcode_system RSI::RapidCode)

# controller_manager discovers SystemInterface plugins through this description.
pluginlib_export_plugin_description_file(hardware_interface rapidcode_system.xml)

install(TARGETS rapidcode_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(rapidcode_system)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle rapidcode_trajectory_transfer)

# --- tests ------------------------------------------------------------------
# Unit tests drive the hardware purely through its exported ros2_control
# interfaces (no rmp / motion) to verify the trajectory_transfer mailbox decode
# and the slot-major payload layout. They link the plugin lib (which pulls in
# RSI::RapidCode transitively) but never call on_configure, so no firmware runs.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rapidcode_passthrough_trajectory_controller REQUIRED)

  ament_add_gtest(test_rapidcode_system_hardware
    test/test_rapidcode_system_hardware.cpp)
  target_link_libraries(test_rapidcode_system_hardware rapidcode_system)
  ament_target_dependencies(test_rapidcode_system_hardware
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    rapidcode_trajectory_transfer)

  # Integration test: drives the full lifecycle (on_configure/on_activate) against a
  # live phantom RMP and streams a real MovePVT move. SKIP_TEST => always compiled
  # (CI catches breakage) but NOT auto-run by `colcon test` -- it needs exclusive
  # ownership of the rmp, so run it by hand with the bringup stopped:
  #   ./build/rapidcode_system/test_rapidcode_system_motion
  ament_add_gtest(test_rapidcode_system_motion
    test/test_rapidcode_system_motion.cpp
    SKIP_TEST)
  target_link_libraries(test_rapidcode_system_motion rapidcode_system)
  ament_target_dependencies(test_rapidcode_system_motion
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    rapidcode_trajectory_transfer)

  # Integration test: feeds controller-generated trajectory_transfer chunks into
  # RapidCodeSystemHardware, so the only motion source is the passthrough
  # controller's dense PVAJ output. Manual for the same RMP ownership reason as
  # test_rapidcode_system_motion.
  ament_add_gtest(test_rapidcode_system_passthrough_controller_motion
    test/test_rapidcode_system_passthrough_controller_motion.cpp
    SKIP_TEST)
  target_link_libraries(test_rapidcode_system_passthrough_controller_motion rapidcode_system)
  ament_target_dependencies(test_rapidcode_system_passthrough_controller_motion
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    rapidcode_trajectory_transfer
    rapidcode_passthrough_trajectory_controller)
endif()

ament_package()
