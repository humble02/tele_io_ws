# marvin_bringup

`marvin_bringup` owns Marvin launch files, runtime YAML, the zero-position
helper, and the manual joint-control UI. It does not link the Marvin SDK
directly; the actual controller connection is owned by `marvin_driver`.

Chinese translation: [README_zh.md](README_zh.md).

## Build

From the workspace root:

```bash
cd /home/ccs/ros2/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select marvin_driver marvin_bringup
source install/setup.bash
```

Do not build ROS packages from a conda environment.

## Driver Launch Files

Before starting real hardware, confirm the controller IP is reachable and no
other SDK client is connected. The default controller IP is `192.168.1.190`;
override it with `robot_ip:=...` when needed.

Position mode:

```bash
ros2 launch marvin_bringup marvin_position.launch.py robot_ip:=192.168.1.190 arms:=both
```

Joint impedance mode:

```bash
ros2 launch marvin_bringup marvin_impedance.launch.py robot_ip:=192.168.1.190 arms:=both
```

Joint impedance PD-feedforward mode:

```bash
ros2 launch marvin_bringup marvin_impedance_pd.launch.py robot_ip:=192.168.1.190 arms:=both
```

Current Vive arm teleoperation testing prefers regular joint impedance mode:

```bash
ros2 launch robot_bringup bringup_real.launch.py \
  marvin_launch:=marvin_impedance.launch.py
```

Useful launch arguments:

```text
namespace:=marvin
robot_ip:=192.168.1.190
arms:=both | left | right
auto_connect:=true | false
velocity_ratio:=10
acceleration_ratio:=10
```

## Zero-Position Helper

Run the zero-position helper only after the driver is running and
`/marvin/*/joint_states` is publishing steadily:

```bash
ros2 launch marvin_bringup marvin_zero.launch.py arms:=both
```

`marvin_zero_position_node` waits for feedback, initializes the first command
from the measured joint position, and then interpolates to zero. It is not
started automatically by the driver.

Common zeroing arguments:

```text
namespace:=marvin
arms:=both | left | right
command_rate_hz:=50.0
hold_before_move_sec:=0.5
move_duration_sec:=5.0
timeout_sec:=30.0
tolerance_rad:=0.02
exit_on_success:=true
```

## Manual Joint-Control UI

Run the Tk manual UI:

```bash
ros2 run marvin_bringup marvin_joint_control_ui.py
```

The UI subscribes to:

```text
/marvin/left/joint_states
/marvin/right/joint_states
```

and publishes:

```text
/marvin/left/joint_commands
/marvin/right/joint_commands
```

It refuses to publish before feedback by default. Install `python3-tk` if the
Tkinter import fails.

Useful UI parameters:

```text
arms:=both | left | right
topic_namespace:=marvin
live_rate_hz:=20.0
require_feedback:=true
slider_min_deg:=-180.0
slider_max_deg:=180.0
ui_scale:=1.6
window_width:=1320
window_height:=600
```

Example:

```bash
ros2 run marvin_bringup marvin_joint_control_ui.py --ros-args \
  -p arms:=right \
  -p require_feedback:=true
```

## Step-by-Step Hardware Startup

For first hardware tests, use one arm and a slow zeroing motion.

Terminal 1: start only the Marvin driver in position mode:

```bash
cd /home/ccs/ros2/teleop_ws
source install/setup.bash
ros2 launch marvin_bringup marvin_position.launch.py \
  robot_ip:=192.168.1.190 \
  arms:=left \
  auto_connect:=true
```

Terminal 2: confirm feedback:

```bash
cd /home/ccs/ros2/teleop_ws
source install/setup.bash
ros2 topic echo --once /marvin/left/joint_states
ros2 topic hz /marvin/left/joint_states
```

Terminal 2: run slow zeroing after feedback is valid:

```bash
ros2 launch marvin_bringup marvin_zero.launch.py \
  arms:=left \
  move_duration_sec:=15.0
```

Use both arms only after single-arm tests are safe.

## Hardware Notes

- Confirm emergency stop, enable state, power, air supply, and external devices
  are in the expected state. Keep the arm workspace clear.
- Confirm MarvinPlatform_EN, SDK demos, check nodes, and other direct SDK
  clients are stopped before starting driver launch files.
- The arms should be stationary before switching position, joint impedance, or
  PD-feedforward modes.
- Tune `joint_impedance_k` and `joint_impedance_d` in YAML first. Do not persist
  controller parameters unless explicitly requested.
- `/joint_states` from `robot_bringup` is for whole-robot display only. Use
  `/marvin/*/joint_states` for driver feedback checks.
