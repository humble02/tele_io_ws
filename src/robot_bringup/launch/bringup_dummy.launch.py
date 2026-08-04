from __future__ import annotations

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from robot_bringup_common import append_visualization_nodes


def launch_setup(context, *args, **kwargs):
    nodes = [
        Node(
            package="robot_bringup",
            executable="dummy_driver",
            name="dummy_driver",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "publish_rate_hz": ParameterValue(
                        LaunchConfiguration("dummy_publish_rate_hz"), value_type=float
                    ),
                    "arms": LaunchConfiguration("arms"),
                    "hands": LaunchConfiguration("hands"),
                    "initial_arm_pose": LaunchConfiguration("initial_arm_pose"),
                }
            ],
        )
    ]
    return append_visualization_nodes(context, nodes)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "whole_robot_state",
                default_value="true",
                description="Publish global /joint_states and whole-robot TF",
            ),
            DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("stale_timeout_sec", default_value="0.0"),
            DeclareLaunchArgument("dummy_publish_rate_hz", default_value="1000.0"),
            DeclareLaunchArgument("arms", default_value="both"),
            DeclareLaunchArgument("hands", default_value="both"),
            DeclareLaunchArgument(
                "initial_arm_pose", default_value="zero", choices=["zero", "elbow"]
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
