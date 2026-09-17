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

"""Bring up the elfin5 (6-axis) RapidCode ros2_control demo.

Same shape as two_axis.launch.py, but the URDF declares the 6 elfin joints
(elfin_joint1 -> axis 0 ... elfin_joint6 -> axis 5) which the rapidcode_system
plugin binds into a SINGLE RapidCode::MultiAxis. The passthrough trajectory
controller streams a trajectory's points to the plugin (MultiAxis::MovePVT).

use_hardware:=false -> RapidCode phantom axes (no EtherCAT hardware).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_hardware = LaunchConfiguration("use_hardware")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rapidcode_bringup"),
                    "urdf",
                    "elfin5_rapidcode.urdf.xacro",
                ]
            ),
            " use_hardware:=",
            use_hardware,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("rapidcode_bringup"), "config", "elfin5_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="both",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    passthrough_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rapidcode_passthrough_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_hardware",
                default_value="false",
                description="false => RapidCode phantom axes (no EtherCAT hardware).",
            ),
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            passthrough_controller_spawner,
        ]
    )
