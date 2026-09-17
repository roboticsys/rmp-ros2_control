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

"""Launch the MoveIt move_group server for the elfin5 (6-axis) RapidCode robot.

Run this ALONGSIDE rapidcode_bringup elfin5.launch.py (which owns the
controller_manager, joint_state_broadcaster,
rapidcode_passthrough_trajectory_controller, and robot_state_publisher).
move_group only plans and dispatches execution to the existing passthrough
trajectory controller via its follow_joint_trajectory action; it does not start
controllers or publish /robot_description to a topic, so there is no conflict
with the bringup.

The robot_description is the SAME xacro the bringup uses, loaded directly from
rapidcode_bringup so MoveIt's model is byte-identical to the running system
(use_hardware:=false -- move_group ignores the <ros2_control> block, it only needs
the geometry).
"""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


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
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # OMPL stays the default single-goal planner; the Pilz pipeline adds
        # PTP/LIN/CIRC and the MoveGroupSequence capability used to blend a
        # multi-pose "recipe" into ONE continuous trajectory (no stops at interior
        # waypoints). Enabling pilz makes to_moveit_configs() auto-load
        # config/pilz_cartesian_limits.yaml, which must exist.
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        # The bringup's robot_state_publisher already owns /robot_description and
        # /tf, so don't republish the description from move_group.
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    # Serve the Pilz MoveGroupSequence action + service (/sequence_move_group) so a
    # blended multi-pose sequence can be planned and executed. generate_move_group_launch
    # reads this as the default of the `capabilities` launch arg -> move_group's
    # `capabilities` parameter (space-separated list of extra capabilities).
    moveit_config.move_group_capabilities["capabilities"] = (
        "pilz_industrial_motion_planner/MoveGroupSequenceAction "
        "pilz_industrial_motion_planner/MoveGroupSequenceService"
    )

    return generate_move_group_launch(moveit_config)
