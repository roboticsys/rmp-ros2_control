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

"""Bring up the two-axis RapidCode ros2_control smoke test (no robot model).

The URDF declares two generic joints (joint1 -> axis 0, joint2 -> axis 1) which
the rapidcode_system plugin binds into a SINGLE RapidCode::MultiAxis. The
passthrough trajectory controller streams the trajectory's points to the plugin,
which feeds them to MultiAxis::MovePVT, exercising the same chunked streaming
path the elfin5 bringup uses. Drive it with send_passthrough_trajectory.py.
(forward_position_controller is loaded inactive as a minimal alternative.)
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
                    "rapidcode_two_axis.urdf.xacro",
                ]
            ),
            " use_hardware:=",
            use_hardware,
        ]
    )
    # Wrap in ParameterValue(..., value_type=str) so launch passes the URDF as a
    # string parameter instead of trying to parse it as YAML.
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("rapidcode_bringup"), "config", "rapidcode_two_axis_controllers.yaml"]
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

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
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
            forward_position_controller_spawner,
        ]
    )
