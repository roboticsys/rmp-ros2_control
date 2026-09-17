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

"""Launch RViz with the MoveIt MotionPlanning panel for the elfin5 (6-axis) robot.

Run this in the GUI sidecar (compose `gui` profile, on the host X display). It
loads config/moveit.rviz and the MoveIt parameters RViz needs (semantic
description + kinematics) so you can set joint-space or Cartesian goals, Plan, and
Execute. Bring up elfin5.launch.py + move_group.launch.py first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    bringup_share = get_package_share_directory("rapidcode_bringup")
    urdf_path = os.path.join(
        bringup_share, "urdf", "elfin5_rapidcode.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "elfin5", package_name="rapidcode_moveit_config"
        )
        .robot_description(
            file_path=urdf_path, mappings={"use_hardware": "false"}
        )
        .robot_description_semantic(file_path="config/elfin5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        # Match move_group.launch.py so RViz's MotionPlanning panel lists the Pilz
        # pipeline (pick PTP/LIN/CIRC in its planner dropdown). Enabling pilz makes
        # to_moveit_configs() require config/pilz_cartesian_limits.yaml.
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    return generate_moveit_rviz_launch(moveit_config)
