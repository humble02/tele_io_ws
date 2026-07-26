# vive_marvin_teleop

`vive_marvin_teleop` maps Vive chest/wrist tracker TF into Marvin arm TCP
targets, solves IK with the vendored Marvin kinematics SDK, and publishes
per-arm joint commands.

This package is a teleoperation algorithm package. It does not connect to
SteamVR and does not connect to the Marvin controller directly.

## IO End-Effector Pose Control

The package also provides `io_marvin_teleop`. It ignores IO-provided joint solutions and
subscribes to `/io_teleop/target_ee_poses` as `geometry_msgs/msg/PoseArray`. The two poses
are passed to the vendored Marvin IK as Tianji SDK TCP targets and emitted on the normal
left and right Marvin joint command topics.

The current IO order is `poses[0]` for the right arm and `poses[1]` for the left arm.
Each pose is expressed in the common upper-body base frame, with translation in meters.
The node applies the arm mount transforms from the whole-robot URDF to obtain each Tianji
SDK arm-base target, then uses `position_scale: 0.8` to retarget the human/source workspace
to Marvin arm reach. The order, mounts, and scale are configured in
`config/io_marvin_teleop.yaml`.

```bash
ros2 launch vive_marvin_teleop io_marvin_teleop.launch.py
ros2 service call /io_marvin_teleop/set_enabled std_srvs/srv/SetBool "{data: true}"
```

The node is disabled by default. Tianji IK uses the left and right elbow-pose targets as
fixed reference joints. Marvin feedback is used only for timeout protection and to
initialize command limiting. The node does not consume `/io_teleop/joint_cmd`. Do not
enable another Marvin command publisher at the same time.

## Tracker Roles

The current three-tracker setup is:

```yaml
left_wrist:  "LHR-E651F39A"
right_wrist: "LHR-F757F16E"
chest:       "LHR-F854821A"
```

`vive_openvr` publishes these as TF frames under `vive_world`:

```text
vive_world -> vive/chest
vive_world -> vive/left_wrist
vive_world -> vive/right_wrist
```

## Data Flow

```text
vive_openvr TF
  -> static chest and wrist alignment TF
  -> left_chest/right_chest reference frames
  -> tianji_left/tianji_right SDK TCP target frames
  -> Marvin IK
  -> /marvin/left/joint_commands
  -> /marvin/right/joint_commands
```

Default command outputs:

```text
/marvin/left/joint_commands
/marvin/right/joint_commands
```

Default feedback inputs:

```text
/marvin/left/joint_states
/marvin/right/joint_states
```

The node starts disabled and publishes no commands until explicitly enabled.

## Coordinate Frames

`vive_openvr` provides all tracker poses in SteamVR standing space. This package
does not treat `vive_world` as the robot base. Instead, it uses a chest tracker
as the operator body reference and publishes static alignment frames:

```text
vive/chest -> left_chest_base -> left_chest
vive/chest -> right_chest_base -> right_chest
vive/left_wrist -> tianji_left
vive/right_wrist -> tianji_right
```

Meaning:

- `vive/chest`: raw chest tracker frame after the role correction in
  `vive_openvr`.
- `left_chest_base` and `right_chest_base`: side-specific chest mount offsets
  and rotations used to align the operator chest frame to the Marvin IK
  convention.
- `left_chest` and `right_chest`: reference frames used as the IK base frame
  for each arm.
- `vive/left_wrist` and `vive/right_wrist`: raw wrist tracker frames after role
  correction.
- `tianji_left` and `tianji_right`: target TCP frames sent to the Marvin SDK IK.

The control transform is:

```text
left_chest  -> tianji_left
right_chest -> tianji_right
```

The Marvin IK target is the SDK TCP/flange convention used by the official
Tianji arm code path. It is not the Wuji hand palm URDF link. If the tracker
mount direction or desired tool point is wrong, adjust
`config/static_transforms.yaml`, especially `wrist_to_tianji`, rather than
adding rotations inside the control loop.

## Launch Order

Start robot-side bringup first. Dummy mode is recommended before real hardware:

```bash
cd /home/ccs/ros2/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup bringup_dummy.launch.py
```

Real Marvin hardware currently prefers joint impedance mode for live arm
teleoperation:

```bash
ros2 launch robot_bringup bringup_real.launch.py \
  marvin_launch:=marvin_impedance.launch.py
```

Start Vive input:

```bash
ros2 launch vive_openvr vive_openvr.launch.py rviz:=true
```

Start arm teleoperation:

```bash
ros2 launch vive_marvin_teleop vive_marvin_teleop.launch.py
```

Enable command output after checking feedback and TF:

```bash
ros2 service call /vive_marvin_teleop/set_enabled std_srvs/srv/SetBool "{data: true}"
```

Disable command output:

```bash
ros2 service call /vive_marvin_teleop/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## Checks

Tracker data:

```bash
ros2 run vive_openvr list_trackers --all
ros2 topic echo --once /vive/chest/pose
ros2 topic echo --once /vive/left_wrist/pose
ros2 topic echo --once /vive/right_wrist/pose
```

TF chain:

```bash
ros2 run tf2_ros tf2_echo vive/chest right_chest
ros2 run tf2_ros tf2_echo left_chest tianji_left
ros2 run tf2_ros tf2_echo right_chest tianji_right
```

Marvin feedback and commands:

```bash
ros2 topic hz /marvin/right/joint_states
ros2 topic hz /marvin/right/joint_commands
ros2 topic echo /marvin/right/joint_commands
```

If all three tracker pose topics exist but arm command topics have no data,
check these in order:

1. `vive_marvin_teleop` is enabled with `/vive_marvin_teleop/set_enabled`.
2. Fresh `/marvin/left/joint_states` and `/marvin/right/joint_states` feedback exists.
3. `left_chest -> tianji_left` and `right_chest -> tianji_right` TF exist.
4. IK is not reporting target out of range or joint limit exceeded.

## Configuration Files

- `config/vive_marvin_teleop.yaml`: control rate, feedback topics, command
  topics, TF names, IK branch parameters, singularity tolerance, and joint step
  limit.
- `config/static_transforms.yaml`: chest-side alignment and wrist-to-Tianji TCP
  transforms.
- `config/ccs_m6.MvKDCfg`: Marvin kinematics SDK configuration.

Useful runtime parameters in `vive_marvin_teleop.yaml`:

```text
control_rate_hz: 60.0
auto_enable: false
feedback_timeout_sec: 0.5
tf_timeout_sec: 0.5
max_joint_step_rad: 0.02
command_publish_on_change_only: false
```

`max_joint_step_rad` limits how much each published joint command can move per
control tick. Keep it conservative during early alignment work.

## Tracker Completeness

The normal configuration uses all three tracker roles: `chest`, `left_wrist`,
and `right_wrist`. Treat missing tracker TF as a fault during full arm
teleoperation, not as an expected operating mode.

The current node initializes both arms and checks fresh feedback for both arms
when enabling. If one wrist TF is missing after enable, that arm is skipped and
the node logs a throttled TF warning. Restore the missing tracker pose before
continuing real hardware teleoperation.

## Safety Notes

- The node starts disabled by default. Keep `auto_enable: false` for real
  hardware.
- Always confirm `/marvin/*/joint_states` before enabling command output.
- Stop command output with `/vive_marvin_teleop/set_enabled` before stopping the
  Marvin driver.
- Validate TF in RViz before real motion. A wrong chest rotation or
  `wrist_to_tianji` rotation can command a large unexpected Cartesian target.
- The first deployed alignment will probably need manual rotation tuning in
  `static_transforms.yaml`. Make those corrections in the static TF layer so
  the IK node interface remains stable.
