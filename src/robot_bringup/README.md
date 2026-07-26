# robot_bringup

`robot_bringup` starts the combined Marvin arms and Wuji hands in either real-driver or
dummy-driver mode, then publishes the whole-robot visualization state.

The package provides `scripts/joint_state_aggregator.py`, a Python ROS node installed as
`joint_state_aggregator`. It subscribes to per-device joint-state topics and publishes one
global `/joint_states` stream for `robot_state_publisher`.

The default joint list matches `robot_description/urdf/robot.xacro`: 14 Marvin arm joints
plus 40 Wuji hand joints. `/joint_states` is only the whole-robot visualization stream;
hardware and dummy drivers should keep publishing their own per-device topics.

Default inputs:

```text
/marvin/left/joint_states
/marvin/right/joint_states
/hand_left/joint_states
/hand_right/joint_states
```

If an input topic is missing, those joints stay at zero. When a topic later appears, the
matching joint names override the zero values.

## Real Bringup

Start the real low-level drivers for both arms and both hands, plus aggregation,
whole-robot `robot_state_publisher`, and RViz:

```bash
ros2 launch robot_bringup bringup_real.launch.py
```

`bringup_real.launch.py` includes `marvin_bringup` and
`wujihand_bringup/wujihand_dual_driver.launch.py`. The Marvin driver defaults to `/marvin`
with `arms:=both`. Wuji hands are discovered by USB serial number and each driver publishes
under `/hand_left` or `/hand_right` after detecting physical handedness.

Use another Marvin mode launch explicitly when needed:

```bash
ros2 launch robot_bringup bringup_real.launch.py \
  marvin_launch:=marvin_impedance.launch.py
```

## Dummy Bringup

Start a fake low-level driver for both arms and both hands, plus the same aggregation and
visualization:

```bash
ros2 launch robot_bringup bringup_dummy.launch.py
```

`dummy_driver` subscribes to the normal command topics and immediately republishes those
commands as device-scoped state feedback:

```text
/marvin/left/joint_commands   -> /marvin/left/joint_states
/marvin/right/joint_commands  -> /marvin/right/joint_states
/hand_left/joint_commands     -> /hand_left/joint_states
/hand_right/joint_commands    -> /hand_right/joint_states
```

This lets real teleoperation input and mapping nodes move the RViz robot without arm or hand
hardware.

## IO Teleop Joint Bridge

Start the IO compatibility bridge after either dummy or real Marvin bringup:

```bash
ros2 launch robot_bringup io_joint_state_bridge.launch.py
```

The bridge publishes Marvin arm feedback plus Wuji hand feedback as one
54-joint `sensor_msgs/msg/JointState`:

```text
/marvin/left/joint_states + /marvin/right/joint_states
  + /hand_left/joint_states + /hand_right/joint_states
  -> /io_teleop/joint_states
```

It also forwards arm and hand `sensor_msgs/msg/JointState` commands:

```text
/io_teleop/joint_cmd -> /marvin/left/joint_commands + /marvin/right/joint_commands
/io_teleop/joint_cmd_finger_left -> /hand_left/joint_commands
/io_teleop/joint_cmd_finger_right -> /hand_right/joint_commands
```

The default IO state order is `Joint1_R` through `Joint7_R`, `Joint1_L` through
`Joint7_L`, then the 20 left-hand joints and 20 right-hand joints. The bridge maps
commands by joint name when the incoming `JointState.name` field is populated.
For unnamed commands, `io_command_joint_names` defines the arm positional order;
hand commands use the default 20-joint order for the corresponding hand.

## Marvin Elbow Pose

Move the Marvin arms from the current measured feedback to a bent-elbow pose:

```bash
ros2 launch robot_bringup marvin_elbow_pose.launch.py
```

Default targets are in degrees:

```text
left:  [-90,  90, 90, -90, 0, 0, 0]
right: [-90, -90, 90,  90, 0, 0, 0]
```

The node waits for `/marvin/left/joint_states` and `/marvin/right/joint_states`,
captures the current joint positions, then publishes a smooth interpolated
trajectory to `/marvin/left/joint_commands` and `/marvin/right/joint_commands`.
Use `arms:=left` or `arms:=right` to move only one side.

Run either bringup without RViz:

```bash
ros2 launch robot_bringup bringup_dummy.launch.py rviz:=false
ros2 launch robot_bringup bringup_real.launch.py rviz:=false
```

Disable real hand drivers when testing arms only:

```bash
ros2 launch robot_bringup bringup_real.launch.py hands:=false
```

## Runtime Notes

- `robot_bringup` does not launch teleoperation input packages or algorithms.
  Start `wuji_glove`, `vive_openvr`, `wujihand_teleop`, and
  `vive_marvin_teleop` in separate terminals when teleoperation is needed.
- The global `/joint_states` stream is for `robot_state_publisher` and RViz.
  Low-level drivers and teleoperation algorithms should use device-scoped topics
  such as `/marvin/right/joint_states` and `/hand_right/joint_commands`.
- Dummy bringup mirrors command topics into feedback topics. This is useful for
  visualization and topic-chain checks, but it does not validate real motor
  mode, controller limits, or hardware timing.
- Real bringup defaults to Marvin position mode. For Vive arm teleoperation,
  currently prefer `marvin_launch:=marvin_impedance.launch.py`.
- `marvin_elbow_pose` publishes motion commands. Start the correct dummy or real
  Marvin bringup first, verify feedback is present, and keep other command
  publishers stopped unless intentionally testing arbitration.
- For arm-only testing on real hardware, use `hands:=false` so the Wuji hand
  drivers are not started. For hand-only testing, keep arm teleoperation
  disabled and ignore `/marvin/*` command topics.
- If RViz does not move, check the per-device feedback first, then check
  `/joint_states`. Missing device feedback leaves that part of the composed
  model at zero.
