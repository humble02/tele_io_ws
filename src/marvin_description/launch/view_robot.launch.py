from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("marvin_description"), "rviz", "view_robot.rviz"]
    )

    robot_description_path = PathJoinSubstitution(
        [
            FindPackageShare("marvin_description"),
            "urdf",
            "Marvin M6-S-L-CCS-696-V4.0 urdf.urdf",
        ]
    )
    with open(robot_description_path.perform(context), "r", encoding="utf-8") as urdf_file:
        robot_description = {"robot_description": urdf_file.read()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
