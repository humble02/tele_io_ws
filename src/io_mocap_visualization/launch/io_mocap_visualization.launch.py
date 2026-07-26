import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'io_mocap_visualization'
    urdf_file_name = 'blender_human.urdf'

    # 获取路径
    pkg_share = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    # 读取 URDF 内容
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }]
        ),

        Node(
            package=pkg_name,
            executable='tf_relay',
            name='tf_relay_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'io_mocap_visualization.rviz')]
        ),
    ])
