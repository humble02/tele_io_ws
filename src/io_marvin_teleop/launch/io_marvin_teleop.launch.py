from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pkg_file(*parts: str) -> str:
    return str(Path(get_package_share_directory("io_marvin_teleop")).joinpath(*parts))


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=_pkg_file("config", "io_marvin_teleop.yaml")
            ),
            DeclareLaunchArgument(
                "kine_config", default_value=_pkg_file("config", "ccs_m6.MvKDCfg")
            ),
            Node(
                package="io_marvin_teleop",
                executable="io_marvin_teleop_node",
                name="io_marvin_teleop",
                output="screen",
                emulate_tty=True,
                arguments=[
                    "--config",
                    LaunchConfiguration("config"),
                    "--kine-config",
                    LaunchConfiguration("kine_config"),
                ],
            ),
        ]
    )
