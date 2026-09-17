cmake_minimum_required(VERSION 3.16)
project(rapidcode_moveit_config)

find_package(ament_cmake REQUIRED)

# config/ holds the SRDF + the MoveIt YAMLs + moveit.rviz; launch/ holds the
# move_group and RViz launches. Nothing to compile -- just installed assets.
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
