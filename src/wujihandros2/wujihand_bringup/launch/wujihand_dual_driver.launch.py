import os
import sys

from launch import LaunchDescription
from launch import logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from common import list_serial_numbers

_logger = logging.get_logger("wujihand_dual_driver")


def spawn_dual_hand_drivers(context):
    serial_numbers = list_serial_numbers()
    if not serial_numbers:
        _logger.error("No WujiHand devices found. Is the hardware connected?")
        return []

    _logger.info(f"Discovered {len(serial_numbers)} device(s): {', '.join(serial_numbers)}")

    actions = []
    for index, serial_number in enumerate(serial_numbers):
        actions.append(
            Node(
                package="wujihand_driver",
                executable="wujihand_driver_node",
                name="wujihand_driver",
                namespace=f"hand_{index}",
                parameters=[
                    {
                        "serial_number": serial_number,
                        "name_by_handedness": True,
                        "publish_rate": ParameterValue(
                            LaunchConfiguration("publish_rate"), value_type=float
                        ),
                        "filter_cutoff_freq": ParameterValue(
                            LaunchConfiguration("filter_cutoff_freq"), value_type=float
                        ),
                        "diagnostics_rate": ParameterValue(
                            LaunchConfiguration("diagnostics_rate"), value_type=float
                        ),
                    }
                ],
                output="screen",
                emulate_tty=True,
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_rate",
                default_value="1000.0",
                description="Joint state publish rate in Hz",
            ),
            DeclareLaunchArgument(
                "filter_cutoff_freq",
                default_value="10.0",
                description="Low-pass filter cutoff frequency in Hz",
            ),
            DeclareLaunchArgument(
                "diagnostics_rate",
                default_value="10.0",
                description="Diagnostics publish rate in Hz",
            ),
            OpaqueFunction(function=spawn_dual_hand_drivers),
        ]
    )
