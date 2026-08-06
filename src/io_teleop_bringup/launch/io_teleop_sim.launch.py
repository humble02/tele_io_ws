from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def package_launch(package: str, filename: str, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("arms", default_value="both"),
            DeclareLaunchArgument("hands", default_value="both"),
            DeclareLaunchArgument(
                "arm_command_mode",
                default_value="target_pose",
                choices=["joint_cmd", "target_pose"],
            ),
            DeclareLaunchArgument(
                "initial_arm_pose", default_value="elbow", choices=["zero", "elbow"]
            ),
            package_launch(
                "robot_bringup",
                "bringup_dummy.launch.py",
                {
                    "rviz": LaunchConfiguration("rviz"),
                    "arms": LaunchConfiguration("arms"),
                    "hands": LaunchConfiguration("hands"),
                    "initial_arm_pose": LaunchConfiguration("initial_arm_pose"),
                },
            ),
            package_launch(
                "io_joint_state_bridge",
                "io_joint_state_bridge.launch.py",
                {
                    "enable_marvin_limit_promotion": "false",
                    "forward_arm_commands": PythonExpression(
                        ["'", LaunchConfiguration("arm_command_mode"), "' == 'joint_cmd'"]
                    ),
                    "enable_footswitch": PythonExpression(
                        ["'", LaunchConfiguration("arm_command_mode"), "' == 'joint_cmd'"]
                    ),
                },
            ),
            package_launch(
                "io_marvin_teleop",
                "io_marvin_teleop.launch.py",
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("arm_command_mode"), "' == 'target_pose'"]
                    )
                ),
            ),
        ]
    )
