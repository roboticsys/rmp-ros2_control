cmake_minimum_required(VERSION 3.16)
project(rapidcode_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

# Motion scripts (rclpy). Installed into lib/<pkg> so `ros2 run` finds them.
install(
  PROGRAMS
    scripts/send_passthrough_trajectory.py
    scripts/send_pilz_sequence.py
    scripts/send_pilz_cartesian_sequence.py
    scripts/send_pilz_cartesian_recipe.py
    scripts/send_cartesian_path.py
    scripts/send_cartesian_path_streamed.py
    scripts/print_tcp_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
