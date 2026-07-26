# marvin_ros2

`src/marvin_ros2` contains the Marvin dual-arm ROS 2 integration used by this
workspace. It is a container directory for the low-level driver package and the
bringup/operator-tool package.

Chinese translation: [README_zh.md](README_zh.md).

## Packages

- [`marvin_driver`](marvin_driver/README.md): C++ ROS 2 wrapper around the old
  Tianji Marvin controller SDK. It owns the controller TCP connection, links the
  vendored SDK, publishes arm feedback, accepts joint command topics, and exposes
  connect/release/estop services.
- [`marvin_bringup`](marvin_bringup/README.md): launch files, runtime YAML,
  zero-position helper, and manual joint-control UI for running the driver in
  position, joint impedance, or joint impedance PD-feedforward mode.

## SDK Scope

This integration targets the old controller SDK under:

```bash
/home/ccs/ros2/TJ_FX_ROBOT_CONTRL_SDK/contrlSDK
```

The selected SDK headers and `libMarvinSDK.so` are vendored in
`marvin_driver/vendor/contrlSDK`. Treat those files as copies of the selected
SDK build output; do not edit them in place unless a migration note explains the
change.

## Workspace Role

- Marvin arm feedback remains device-scoped:

  ```text
  /marvin/left/joint_states
  /marvin/right/joint_states
  ```

- Marvin arm commands are accepted on:

  ```text
  /marvin/left/joint_commands
  /marvin/right/joint_commands
  ```

- The Marvin driver does not publish the global `/joint_states`. Whole-robot
  visualization is handled by `robot_bringup`.
- Teleoperation input and algorithm packages are launched separately from Marvin
  hardware bringup.

## Quick Entry Points

Build the Marvin packages:

```bash
cd /home/ccs/ros2/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select marvin_driver marvin_bringup
source install/setup.bash
```

Start the real driver in the currently preferred teleoperation mode:

```bash
ros2 launch marvin_bringup marvin_impedance.launch.py
```

Run direct SDK check nodes only when `marvin_driver_node` is stopped:

```bash
ros2 run marvin_driver marvin_link_check_node --ros-args -p robot_ip:=192.168.1.190
```

For details, read:

- Driver internals and SDK behavior:
  [`marvin_driver/README.md`](marvin_driver/README.md)
- Launch files, zeroing, manual UI, and hardware startup flow:
  [`marvin_bringup/README.md`](marvin_bringup/README.md)
