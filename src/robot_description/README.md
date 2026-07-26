# robot_description

Top-level robot description package for the teleoperation robot. This package owns the final composed model and references the resource packages `marvin_description`, `wuji_description`, and `realsense2_description`.

## Robot Configuration

The composed robot is defined in `urdf/robot.xacro` and expanded to `urdf/robot.urdf`.

`robot_description` is the main description entry point for the workspace. It
composes the resource packages under `src/` into the final robot model:

- `marvin_description`: left and right Marvin M6-S arm URDFs and meshes.
- `wuji_description`: left and right Wuji hand URDFs, meshes, and hand-only MJCF
  resources.
- `realsense2_description`: RealSense camera xacro models and meshes.

The final package adds the shared base, lift platform, hand adapters, camera
adapters, expanded URDF, RViz config, and MuJoCo scene.

Configuration:

- Base: `base_link` chassis plus fixed `lift_platform_link`.
- Arms: two Marvin M6-S 7-DoF arms.
  - Left arm joints: `Joint1_L` to `Joint7_L`.
  - Right arm joints: `Joint1_R` to `Joint7_R`.
- Hands: two Wuji five-finger hands.
  - Each hand has 20 revolute finger joints.
  - Left hand joints use the `left_finger*_joint*` naming pattern.
  - Right hand joints use the `right_finger*_joint*` naming pattern.
- Cameras: one head RealSense D435i and two wrist RealSense D435i cameras.

Actuated model DOF:

- Arms: 14 DoF total.
- Hands: 40 DoF total.
- Total articulated DoF: 54.

The base, lift platform, hand docking links, camera adapter links, and camera frames are fixed links in the robot description.

## Link Frames for Teleoperation Mapping

Use these link frames as stable references for later teleoperation mapping:

- `base_link`: root chassis frame for whole-robot placement.
- `lift_platform_link`: fixed platform frame where both arms are mounted.
- `Base_L`, `Base_R`: left and right arm base frames.
- `Link7_L`, `Link7_R`: left and right arm wrist/flange frames.
- `left_hand_docking_link`, `right_hand_docking_link`: fixed hand docking adapters mounted from each arm wrist.
- `left_palm_link`, `right_palm_link`: hand palm frames, useful as hand task frames.
- `left_camera_adapter_link`, `right_camera_adapter_link`: wrist camera adapter frames mounted from each arm wrist.
- `head_camera_color_optical_frame`: head RGB optical frame.
- `left_wrist_camera_color_optical_frame`, `right_wrist_camera_color_optical_frame`: wrist RGB optical frames.

For ROS optical frames, use the standard camera convention: `+X` right, `+Y` down, `+Z` forward. In the MuJoCo XML, cameras are created under the corresponding `*_color_optical_frame` and rotated back to the OpenGL/MuJoCo camera convention where `+Z` points backward.

The Vive-to-Marvin arm teleoperation node does not consume these URDF links as
its IK target interface. `vive_marvin_teleop` sends transforms named
`left_chest -> tianji_left` and `right_chest -> tianji_right` into the Marvin
SDK IK wrapper. Those `tianji_*` frames follow the SDK TCP/flange convention,
not `left_palm_link` or `right_palm_link`. Use the URDF links for visualization
and calibration reference, then adjust tracker-to-TCP alignment in
`vive_marvin_teleop/config/static_transforms.yaml`.

## Vision

The head camera is a RealSense D435i instance generated from `realsense2_description/urdf/_d435i.urdf.xacro`.

Current head camera mount:

- Parent link: `base_link`.
- Mounted frame: `head_camera_bottom_screw_frame`.
- Position relative to `base_link`: `xyz="0.30 0.0070867 1.4"`.
- Orientation relative to `base_link`: `rpy="0 1.047198 0"`; this is a 60 degree rotation around Y.
- Main RGB optical frame: `head_camera_color_optical_frame`.

The wrist cameras are mounted on:

- `left_camera_adapter_link` as `left_wrist_camera`.
- `right_camera_adapter_link` as `right_wrist_camera`.

## Important Files

- `urdf/robot.xacro`: source xacro for the composed robot.
- `urdf/robot.urdf`: expanded URDF for direct consumers and MuJoCo export.
- `urdf/marvin_left.xacro`, `urdf/marvin_right.xacro`: arm wrapper macros.
- `urdf/wuji_left.xacro`, `urdf/wuji_right.xacro`: hand wrapper macros.
- `meshes/`: base, lift platform, hand docking, and wrist camera adapter meshes.
- `mjcf/robot.xml`: final MuJoCo robot XML.
- `mjcf/robot_raw.xml`: raw MuJoCo export kept for debugging.
- `mjcf/_robot_mujoco_input.urdf`: temporary URDF used for MuJoCo conversion.
- `mjcf/scene.xml`: MuJoCo scene with floor and table.
- `mjcf/scene.py`: OpenCV/MuJoCo renderer for the three robot cameras.

## Build and Run

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_description view_robot.launch.py
```

For zsh:

```bash
source install/setup.zsh
```

Regenerate the expanded URDF from xacro:

```bash
xacro src/robot_description/urdf/robot.xacro > src/robot_description/urdf/robot.urdf
```

Run the MuJoCo scene:

```bash
conda run -n floating python src/robot_description/mjcf/scene.py
```

Render only selected cameras:

```bash
conda run -n floating python src/robot_description/mjcf/scene.py \
  --camera head_camera \
  --camera left_wrist_camera
```
