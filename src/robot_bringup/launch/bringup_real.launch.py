from __future__ import annotations

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(__file__))
from common import append_visualization_nodes


def marvin_bringup_include() -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("marvin_bringup"),
                    "launch",
                    LaunchConfiguration("marvin_launch"),
                ]
            )
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("marvin_namespace"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "arms": LaunchConfiguration("arms"),
            "auto_connect": LaunchConfiguration("auto_connect"),
            "velocity_ratio": LaunchConfiguration("velocity_ratio"),
            "acceleration_ratio": LaunchConfiguration("acceleration_ratio"),
            "feedback_rate_hz": LaunchConfiguration("marvin_feedback_rate_hz"),
        }.items(),
    )


def wujihand_bringup_include() -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("wujihand_bringup"),
                    "launch",
                    "wujihand_dual_driver.launch.py",
                ]
            )
        ),
        launch_arguments={
            "publish_rate": LaunchConfiguration("hand_publish_rate"),
            "filter_cutoff_freq": LaunchConfiguration("hand_filter_cutoff_freq"),
            "diagnostics_rate": LaunchConfiguration("hand_diagnostics_rate"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("hands")),
    )


def launch_setup(context, *args, **kwargs):
    nodes = [marvin_bringup_include(), wujihand_bringup_include()]
    return append_visualization_nodes(context, nodes)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("stale_timeout_sec", default_value="0.0"),
            DeclareLaunchArgument("marvin_launch", default_value="marvin_position.launch.py"),
            DeclareLaunchArgument("marvin_namespace", default_value="marvin"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.1.190"),
            DeclareLaunchArgument("arms", default_value="both"),
            DeclareLaunchArgument("hands", default_value="true"),
            DeclareLaunchArgument("auto_connect", default_value="true"),
            DeclareLaunchArgument("velocity_ratio", default_value="10"),
            DeclareLaunchArgument("acceleration_ratio", default_value="10"),
            DeclareLaunchArgument("marvin_feedback_rate_hz", default_value="1000.0"),
            DeclareLaunchArgument("hand_publish_rate", default_value="1000.0"),
            DeclareLaunchArgument("hand_filter_cutoff_freq", default_value="10.0"),
            DeclareLaunchArgument("hand_diagnostics_rate", default_value="10.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
