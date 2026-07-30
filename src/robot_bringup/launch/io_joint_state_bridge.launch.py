from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


IO_COMMAND_JOINT_NAMES = [
    "Joint1_R",
    "Joint2_R",
    "Joint3_R",
    "Joint4_R",
    "Joint5_R",
    "Joint6_R",
    "Joint7_R",
    "Joint1_L",
    "Joint2_L",
    "Joint3_L",
    "Joint4_L",
    "Joint5_L",
    "Joint6_L",
    "Joint7_L",
]
IO_STATE_JOINT_NAMES = IO_COMMAND_JOINT_NAMES


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("require_both_feedback", default_value="true"),
            DeclareLaunchArgument("io_state_topic", default_value="/io_teleop/joint_states"),
            DeclareLaunchArgument("io_command_topic", default_value="/io_teleop/joint_cmd"),
            DeclareLaunchArgument(
                "io_left_hand_command_topic",
                default_value="/io_teleop/joint_cmd_finger_left",
            ),
            DeclareLaunchArgument(
                "io_right_hand_command_topic",
                default_value="/io_teleop/joint_cmd_finger_right",
            ),
            DeclareLaunchArgument("left_state_topic", default_value="/marvin/left/joint_states"),
            DeclareLaunchArgument("right_state_topic", default_value="/marvin/right/joint_states"),
            DeclareLaunchArgument(
                "left_command_topic", default_value="/marvin/left/joint_commands"
            ),
            DeclareLaunchArgument(
                "right_command_topic", default_value="/marvin/right/joint_commands"
            ),
            DeclareLaunchArgument(
                "left_hand_command_topic", default_value="/hand_left/joint_commands"
            ),
            DeclareLaunchArgument(
                "right_hand_command_topic", default_value="/hand_right/joint_commands"
            ),
            Node(
                package="robot_bringup",
                executable="io_joint_state_bridge",
                name="io_joint_state_bridge",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("publish_rate_hz"), value_type=float
                        ),
                        "require_both_feedback": ParameterValue(
                            LaunchConfiguration("require_both_feedback"), value_type=bool
                        ),
                        "io_command_joint_names": IO_COMMAND_JOINT_NAMES,
                        "io_state_joint_names": IO_STATE_JOINT_NAMES,
                        "io_state_topic": LaunchConfiguration("io_state_topic"),
                        "io_command_topic": LaunchConfiguration("io_command_topic"),
                        "io_left_hand_command_topic": LaunchConfiguration(
                            "io_left_hand_command_topic"
                        ),
                        "io_right_hand_command_topic": LaunchConfiguration(
                            "io_right_hand_command_topic"
                        ),
                        "left_state_topic": LaunchConfiguration("left_state_topic"),
                        "right_state_topic": LaunchConfiguration("right_state_topic"),
                        "left_command_topic": LaunchConfiguration("left_command_topic"),
                        "right_command_topic": LaunchConfiguration("right_command_topic"),
                        "left_hand_command_topic": LaunchConfiguration(
                            "left_hand_command_topic"
                        ),
                        "right_hand_command_topic": LaunchConfiguration(
                            "right_hand_command_topic"
                        ),
                    }
                ],
            ),
        ]
    )
