from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="marvin"),
            DeclareLaunchArgument("arms", default_value="both"),
            DeclareLaunchArgument("command_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("hold_before_move_sec", default_value="0.5"),
            DeclareLaunchArgument("move_duration_sec", default_value="5.0"),
            DeclareLaunchArgument("timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("tolerance_rad", default_value="0.02"),
            DeclareLaunchArgument("exit_on_success", default_value="true"),
            Node(
                package="marvin_bringup",
                executable="marvin_zero_position_node",
                name="marvin_zero_position",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    {
                        "arms": LaunchConfiguration("arms"),
                        "command_rate_hz": ParameterValue(
                            LaunchConfiguration("command_rate_hz"), value_type=float
                        ),
                        "hold_before_move_sec": ParameterValue(
                            LaunchConfiguration("hold_before_move_sec"), value_type=float
                        ),
                        "move_duration_sec": ParameterValue(
                            LaunchConfiguration("move_duration_sec"), value_type=float
                        ),
                        "timeout_sec": ParameterValue(
                            LaunchConfiguration("timeout_sec"), value_type=float
                        ),
                        "tolerance_rad": ParameterValue(
                            LaunchConfiguration("tolerance_rad"), value_type=float
                        ),
                        "exit_on_success": ParameterValue(
                            LaunchConfiguration("exit_on_success"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
