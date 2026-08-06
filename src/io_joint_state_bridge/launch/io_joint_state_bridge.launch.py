from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            DeclareLaunchArgument("publish_rate_hz", default_value="1000.0"),
            DeclareLaunchArgument("require_both_feedback", default_value="true"),
            DeclareLaunchArgument("forward_arm_commands", default_value="true"),
            DeclareLaunchArgument("enable_footswitch", default_value="true"),
            DeclareLaunchArgument("footswitch_device_path", default_value=""),
            DeclareLaunchArgument("footswitch_vendor_id", default_value="13651"),
            DeclareLaunchArgument("footswitch_product_id", default_value="45057"),
            DeclareLaunchArgument("footswitch_grab_device", default_value="true"),
            DeclareLaunchArgument("freeze_mask_topic", default_value="/io_teleop/freeze_mask"),
            DeclareLaunchArgument("hold_publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("hold_feedback_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("resume_arm_max_velocity_rad_s", default_value="0.5"),
            DeclareLaunchArgument("resume_hand_max_velocity_rad_s", default_value="1.0"),
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
                "left_hand_state_topic", default_value="/hand_left/joint_states"
            ),
            DeclareLaunchArgument(
                "right_hand_state_topic", default_value="/hand_right/joint_states"
            ),
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
                package="io_joint_state_bridge",
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
                        "forward_arm_commands": ParameterValue(
                            LaunchConfiguration("forward_arm_commands"), value_type=bool
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
                        "freeze_mask_topic": LaunchConfiguration("freeze_mask_topic"),
                        "hold_publish_rate_hz": ParameterValue(
                            LaunchConfiguration("hold_publish_rate_hz"), value_type=float
                        ),
                        "hold_feedback_timeout_sec": ParameterValue(
                            LaunchConfiguration("hold_feedback_timeout_sec"), value_type=float
                        ),
                        "resume_arm_max_velocity_rad_s": ParameterValue(
                            LaunchConfiguration("resume_arm_max_velocity_rad_s"),
                            value_type=float,
                        ),
                        "resume_hand_max_velocity_rad_s": ParameterValue(
                            LaunchConfiguration("resume_hand_max_velocity_rad_s"),
                            value_type=float,
                        ),
                        "left_hand_state_topic": LaunchConfiguration(
                            "left_hand_state_topic"
                        ),
                        "right_hand_state_topic": LaunchConfiguration(
                            "right_hand_state_topic"
                        ),
                        "left_hand_command_topic": LaunchConfiguration(
                            "left_hand_command_topic"
                        ),
                        "right_hand_command_topic": LaunchConfiguration(
                            "right_hand_command_topic"
                        ),
                    }
                ],
            ),
            Node(
                package="io_joint_state_bridge",
                executable="footswitch_pedal",
                name="footswitch_pedal",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("enable_footswitch")),
                parameters=[
                    {
                        "device_path": LaunchConfiguration("footswitch_device_path"),
                        "vendor_id": ParameterValue(
                            LaunchConfiguration("footswitch_vendor_id"), value_type=int
                        ),
                        "product_id": ParameterValue(
                            LaunchConfiguration("footswitch_product_id"), value_type=int
                        ),
                        "grab_device": ParameterValue(
                            LaunchConfiguration("footswitch_grab_device"), value_type=bool
                        ),
                        "freeze_mask_topic": LaunchConfiguration("freeze_mask_topic"),
                    }
                ],
            ),
        ]
    )
