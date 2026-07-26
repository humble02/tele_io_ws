"""Launch Vive Tracker to Marvin teleoperation and static alignment TFs."""

from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pkg_file(*parts: str) -> str:
    return str(Path(get_package_share_directory("vive_marvin_teleop")).joinpath(*parts))


def _static_tf_node(name: str, parent: str, child: str, args: dict) -> Node:
    cli_args = [
        "--x", str(args.get("x", 0.0)),
        "--y", str(args.get("y", 0.0)),
        "--z", str(args.get("z", 0.0)),
        "--frame-id", parent,
        "--child-frame-id", child,
    ]
    if {"qx", "qy", "qz", "qw"}.issubset(args):
        cli_args.extend([
            "--qx", str(args["qx"]),
            "--qy", str(args["qy"]),
            "--qz", str(args["qz"]),
            "--qw", str(args["qw"]),
        ])
    else:
        cli_args.extend([
            "--roll", str(args.get("roll", 0.0)),
            "--pitch", str(args.get("pitch", 0.0)),
            "--yaw", str(args.get("yaw", 0.0)),
        ])
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=cli_args,
        output="screen",
    )


def _create_static_tf_nodes(context, *args, **kwargs):
    publish = LaunchConfiguration("publish_static_tf").perform(context).lower()
    if publish not in ("true", "1", "yes"):
        return []

    config_path = Path(LaunchConfiguration("static_config").perform(context))
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    chest_frame = LaunchConfiguration("chest_frame").perform(context)
    left_wrist_frame = LaunchConfiguration("left_wrist_frame").perform(context)
    right_wrist_frame = LaunchConfiguration("right_wrist_frame").perform(context)

    nodes = []
    chest = config["chest_mount"]
    shared = chest["shared"]
    for side in ("left", "right"):
        side_cfg = chest[side]
        base = f"{side}_chest_base"
        frame = f"{side}_chest"
        nodes.append(
            _static_tf_node(
                f"{side}_chest_base_tf",
                chest_frame,
                base,
                {
                    "x": shared["x"],
                    "y": shared["y"],
                    "z": shared["z"],
                    "roll": side_cfg["roll"],
                    "pitch": side_cfg["pitch"],
                    "yaw": side_cfg["yaw"],
                },
            )
        )
        nodes.append(
            _static_tf_node(
                f"{side}_chest_tf",
                base,
                frame,
                {
                    "x": side_cfg["local_x"],
                    "y": side_cfg["local_y"],
                    "z": side_cfg["local_z"],
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                },
            )
        )

    wrist_to_tianji = config["wrist_to_tianji"]
    nodes.append(
        _static_tf_node(
            "tianji_left_tf",
            left_wrist_frame,
            "tianji_left",
            wrist_to_tianji["left"],
        )
    )
    nodes.append(
        _static_tf_node(
            "tianji_right_tf",
            right_wrist_frame,
            "tianji_right",
            wrist_to_tianji["right"],
        )
    )
    return nodes


def generate_launch_description() -> LaunchDescription:
    config = LaunchConfiguration("config")
    kine_config = LaunchConfiguration("kine_config")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=_pkg_file("config", "vive_marvin_teleop.yaml")),
            DeclareLaunchArgument("kine_config", default_value=_pkg_file("config", "ccs_m6.MvKDCfg")),
            DeclareLaunchArgument("static_config", default_value=_pkg_file("config", "static_transforms.yaml")),
            DeclareLaunchArgument("publish_static_tf", default_value="true"),
            DeclareLaunchArgument("chest_frame", default_value="vive/chest"),
            DeclareLaunchArgument("left_wrist_frame", default_value="vive/left_wrist"),
            DeclareLaunchArgument("right_wrist_frame", default_value="vive/right_wrist"),
            OpaqueFunction(function=_create_static_tf_nodes),
            Node(
                package="vive_marvin_teleop",
                executable="vive_marvin_teleop_node",
                name="vive_marvin_teleop",
                output="screen",
                emulate_tty=True,
                arguments=["--config", config, "--kine-config", kine_config],
            ),
        ]
    )
