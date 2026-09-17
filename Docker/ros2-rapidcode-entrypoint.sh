#!/bin/bash
# Copyright 2026 Robotic Systems Integration, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Source ROS 2 and the RapidCode workspace overlay, link the user-supplied RSI
# files into place, then exec the command.
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# The repo's inputs/ directory is bind-mounted at /inputs (Docker/compose.yml).
# rmp reads the license and the ENI from /rsi, so point /rsi at the mounted
# copies when they exist. A symlink, not a copy: replacing the file on the
# host takes effect on the next rmp start without recreating the container.
for name in rsi.lic EtherCAT.xml; do
  if [ -f "/inputs/$name" ]; then
    ln -sfn "/inputs/$name" "/rsi/$name"
  fi
done

exec "$@"
