cmake_minimum_required(VERSION 3.16)
project(rapidcode_passthrough_trajectory_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rapidcode_trajectory_transfer REQUIRED)

add_library(${PROJECT_NAME} SHARED src/passthrough_trajectory_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  pluginlib
  control_msgs
  trajectory_msgs
  realtime_tools
  std_srvs
  rapidcode_trajectory_transfer)

# controller_manager discovers ControllerInterface plugins through this file.
pluginlib_export_plugin_description_file(controller_interface
  rapidcode_passthrough_trajectory_controller.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(FILES rapidcode_passthrough_trajectory_controller.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Hermetic controller-output test: no RapidCode, no controller_manager, no action
  # client. It drives update() through fake loaned interfaces and validates the raw
  # trajectory_transfer command doubles the hardware will decode.
  ament_add_gtest(test_passthrough_trajectory_controller_output
    test/test_passthrough_trajectory_controller_output.cpp)
  target_link_libraries(test_passthrough_trajectory_controller_output ${PROJECT_NAME})
  ament_target_dependencies(test_passthrough_trajectory_controller_output
    controller_interface
    hardware_interface
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    control_msgs
    trajectory_msgs
    realtime_tools
    std_srvs
    rapidcode_trajectory_transfer)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  pluginlib
  control_msgs
  trajectory_msgs
  realtime_tools
  std_srvs
  rapidcode_trajectory_transfer)

ament_package()
