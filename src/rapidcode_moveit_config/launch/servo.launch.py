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

"""Launch MoveIt Servo for Elfin5, publishing straight to the passthrough controller.

Run alongside rapidcode_bringup/elfin5.launch.py (start it in phantom mode first:
use_hardware:=false). Servo emits a rolling trajectory_msgs/JointTrajectory on
/rapidcode_passthrough_trajectory_controller/joint_trajectory (set in config/servo.yaml),
which the passthrough controller consumes directly on its ~/joint_trajectory online
intake -- it resamples, splices and synthesizes the decel-to-rest stop-tail internally,
so no servo_trajectory_adapter node is used on this branch (the old /stream/* chunk
protocol is gone).

Drive it with Servo's ~/delta_joint_cmds first, then ~/delta_twist_cmds (Cartesian),
then ~/pose_target_cmds. Servo must be enabled via its /servo_node/start_servo service.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup_share = get_package_share_directory("rapidcode_bringup")
    urdf_path = os.path.join(
        bringup_share, "urdf", "elfin5_rapidcode.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder("elfin5", package_name="rapidcode_moveit_config")
        .robot_description(
            file_path=urdf_path, mappings={"use_hardware": "false"}
        )
        .robot_description_semantic(file_path="config/elfin5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )
    servo_params = {
        "moveit_servo": ParameterBuilder("rapidcode_moveit_config")
        .yaml("config/servo.yaml")
        .to_dict()
    }
    # Required by the Ruckig smoothing plugin. Keep update_period synchronized
    # with moveit_servo.publish_period in config/servo.yaml.
    smoothing_params = {
        "update_period": 0.01,
        "planning_group_name": "elfin_arm",
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            smoothing_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    return LaunchDescription([servo_node])
