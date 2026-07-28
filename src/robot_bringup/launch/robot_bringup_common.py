from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def make_joint_state_aggregator() -> Node:
    config_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "joint_state_aggregator.yaml",
    )
    return Node(
        package="robot_bringup",
        executable="joint_state_aggregator",
        name="joint_state_aggregator",
        output="screen",
        parameters=[
            config_file,
            {
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "stale_timeout_sec": ParameterValue(
                    LaunchConfiguration("stale_timeout_sec"), value_type=float
                ),
            },
        ],
    )


def make_whole_robot_state_publisher(context) -> Node:
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "urdf", "robot.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content.perform(context), value_type=str
        )
    }
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )


def make_rviz_node() -> Node:
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "rviz", "view_robot.rviz"]
    )
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )


def append_visualization_nodes(context, nodes: list[Node]) -> list[Node]:
    nodes.append(make_joint_state_aggregator())
    nodes.append(make_whole_robot_state_publisher(context))

    if LaunchConfiguration("rviz").perform(context).lower() in ("true", "1", "yes"):
        nodes.append(make_rviz_node())

    return nodes
