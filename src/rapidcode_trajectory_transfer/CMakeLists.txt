cmake_minimum_required(VERSION 3.16)
project(rapidcode_trajectory_transfer)

find_package(ament_cmake REQUIRED)

# Header-only interface library: the shared protocol contract. Consumers link
# this target to pull in the include directory; there is nothing to compile.
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
