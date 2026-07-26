from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("marvin_bringup"),
        "config",
        "marvin_position.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="marvin"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.1.190"),
            DeclareLaunchArgument("arms", default_value="both"),
            DeclareLaunchArgument("auto_connect", default_value="true"),
            DeclareLaunchArgument("velocity_ratio", default_value="10"),
            DeclareLaunchArgument("acceleration_ratio", default_value="10"),
            DeclareLaunchArgument("feedback_rate_hz", default_value="1000.0"),
            Node(
                package="marvin_driver",
                executable="marvin_driver_node",
                name="marvin_driver",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    config_file,
                    {
                        "robot_ip": LaunchConfiguration("robot_ip"),
                        "arms": LaunchConfiguration("arms"),
                        "auto_connect": ParameterValue(
                            LaunchConfiguration("auto_connect"), value_type=bool
                        ),
                        "control_mode": "position",
                        "velocity_ratio": ParameterValue(
                            LaunchConfiguration("velocity_ratio"), value_type=int
                        ),
                        "acceleration_ratio": ParameterValue(
                            LaunchConfiguration("acceleration_ratio"), value_type=int
                        ),
                        "feedback_rate_hz": ParameterValue(
                            LaunchConfiguration("feedback_rate_hz"), value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
