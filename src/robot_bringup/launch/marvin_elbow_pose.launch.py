from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("pose", default_value="prepare"),
            DeclareLaunchArgument("arms", default_value="both"),
            DeclareLaunchArgument("command_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("hold_before_move_sec", default_value="0.5"),
            DeclareLaunchArgument("move_duration_sec", default_value="5.0"),
            DeclareLaunchArgument("timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("tolerance_rad", default_value="0.02"),
            DeclareLaunchArgument("exit_on_success", default_value="true"),
            DeclareLaunchArgument("left_state_topic", default_value="/marvin/left/joint_states"),
            DeclareLaunchArgument("right_state_topic", default_value="/marvin/right/joint_states"),
            DeclareLaunchArgument(
                "left_command_topic", default_value="/marvin/left/joint_commands"
            ),
            DeclareLaunchArgument(
                "right_command_topic", default_value="/marvin/right/joint_commands"
            ),
            Node(
                package="robot_bringup",
                executable="marvin_elbow_pose",
                name="marvin_elbow_pose",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "pose": LaunchConfiguration("pose"),
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
                        "left_state_topic": LaunchConfiguration("left_state_topic"),
                        "right_state_topic": LaunchConfiguration("right_state_topic"),
                        "left_command_topic": LaunchConfiguration("left_command_topic"),
                        "right_command_topic": LaunchConfiguration("right_command_topic"),
                    }
                ],
            ),
        ]
    )
