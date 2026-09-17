# ROS 2 Jazzy + RapidCode (ros2_control) development image.
#
# RapidCode is provided by RSI's prebuilt 'rmp' Debian package, exactly as a
# customer install would: it lays down /rsi (libs + RSI.cmake) and the public
# headers in /usr/include. The colcon workspace lives inside the image at
# /ros2_ws and links against that installed package.
#
# Build (an rmp_*_amd64.deb must sit in the repo's inputs/ directory):
#   ./run.sh build            # or, from the repo root: docker build -f Docker/Dockerfile -t rsi/ros2-rapidcode .
# librapidcode.so needs only GLIBC_2.38, so it loads on Ubuntu 24.04 (glibc 2.39).

FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

# ---- ROS 2 build tooling + ros2_control stack --------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo \
      python3-colcon-common-extensions \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-realtime-tools \
      ros-jazzy-rviz2 \
      ros-jazzy-moveit \
      ros-jazzy-moveit-servo \
      ros-jazzy-teleop-twist-keyboard \
      python3-websockets \
      python3-tk \
      xvfb \
      libgl1-mesa-dri \
      mesa-utils \
    && rm -rf /var/lib/apt/lists/*
# moveit-servo drives the online-jog topic of rapidcode_passthrough_trajectory_
# controller (rapidcode_moveit_config servo.launch.py); teleop-twist-keyboard is
# the keyboard twist source for Cartesian jogging (remapped to
# /servo_node/delta_twist_cmds with stamped:=true). Both were previously
# runtime-installed only and vanished on container recreation.

# ---- Install the rmp Debian package ------------------------------------------
# The rmp_*_amd64.deb placed in inputs/ (see inputs/README.md) is installed. Its
# version may carry a branch+commit suffix, hence the glob; ./run.sh build
# checks that exactly one is present. `apt-get install ./file.deb` resolves the .deb's runtime deps
# (libusb-1.0-0, libyubikey0, libfido2-1, libssl3t64, libxdp1, libpcap0.8t64,
# libcap2-bin, ...) and runs its postinst (writes /etc/ld.so.conf.d/rmp.conf +
# ldconfig, setcaps the RT binaries). The systemd service start is skipped
# automatically (no systemctl in a container).
COPY inputs/rmp_*_amd64.deb /tmp/
RUN apt-get update \
    && apt-get install -y /tmp/rmp_*_amd64.deb \
    && rm -f /tmp/rmp_*_amd64.deb \
    && rm -rf /var/lib/apt/lists/*

# ---- Build the colcon workspace ----------------------------------------------
COPY src/ /ros2_ws/src/
RUN source /opt/ros/jazzy/setup.bash \
    && cd /ros2_ws \
    && colcon build --symlink-install

# `docker run` uses the entrypoint (below) to source ROS, but `docker compose
# exec` / `docker exec` do NOT run the entrypoint. Auto-source ROS + the
# workspace for interactive shells (/etc/bash.bashrc) and login shells
# (/etc/profile.d), so `docker compose exec ros2_rapidcode bash` and
# `... bash -lc 'ros2 ...'` work without manual sourcing.
RUN printf '\n# ROS 2 + RapidCode workspace\nsource /opt/ros/jazzy/setup.bash\nsource /ros2_ws/install/setup.bash\n' >> /etc/bash.bashrc \
 && printf '. /opt/ros/jazzy/setup.bash\n. /ros2_ws/install/setup.bash\n' > /etc/profile.d/10-ros2-rapidcode.sh

# ---- RapidCode license and EtherCAT ENI --------------------------------------
# Neither is baked into the image. The repo's inputs/ directory is bind-mounted
# at /inputs (compose.yml) and the entrypoint symlinks /inputs/rsi.lic and
# /inputs/EtherCAT.xml to /rsi/, where rmp expects them. Replacing either file
# on the host therefore needs no rebuild, and the image carries no license.

# ---- Entrypoint: source ROS + the workspace overlay --------------------------
COPY Docker/ros2-rapidcode-entrypoint.sh /ros2-rapidcode-entrypoint.sh
RUN chmod +x /ros2-rapidcode-entrypoint.sh
ENTRYPOINT ["/ros2-rapidcode-entrypoint.sh"]
CMD ["bash"]
