# io_joint_state_bridge

Publishes Marvin arm feedback on `/io_teleop/joint_states` and forwards IO arm
and hand commands to the matching Marvin and Wuji Hand device topics. Hand state
feedback is used for footswitch hold snapshots but is not included in the IO state topic.

```bash
ros2 launch io_joint_state_bridge io_joint_state_bridge.launch.py
```

## PCsensor footswitch interlock

The launch file also starts `footswitch_pedal`, which discovers the
`3553:b001 PCsensor FootSwitch Keyboard` input interface without relying on a
fixed `/dev/input/eventX` number.

- Left pedal (`A`): freeze the left arm and hand at their last forwarded command targets.
- Right pedal (`C`): freeze the right arm and hand.
- Middle pedal (`B`): release all frozen sides and rate-limit the transition back to
  incoming IO joint commands.

Install the input dependency and device permission rule once:

```bash
sudo apt install python3-evdev
sudo install -m 0644 \
  "$(ros2 pkg prefix io_joint_state_bridge)/share/io_joint_state_bridge/udev/99-pcsensor-footswitch.rules" \
  /etc/udev/rules.d/99-pcsensor-footswitch.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reconnect the footswitch after installing the rule. Stop any `evtest --grab`
process before launching, because only one process can exclusively grab the
keyboard interface.

The last forwarded target is retained to avoid changing the position controller's
setpoint when measured position has a steady-state tracking error. Measured feedback
is used only as a startup fallback if no command has been forwarded yet. The bridge
republishes frozen targets at `hold_publish_rate_hz` and uses
`resume_arm_max_velocity_rad_s` / `resume_hand_max_velocity_rad_s` when releasing.
To run the bridge without the device node, pass `enable_footswitch:=false`.

The gate can be tested without hardware after launching with
`enable_footswitch:=false`:

```bash
ros2 topic pub --once /io_teleop/freeze_mask std_msgs/msg/UInt8 '{data: 1}'  # left
ros2 topic pub --once /io_teleop/freeze_mask std_msgs/msg/UInt8 '{data: 2}'  # right
ros2 topic pub --once /io_teleop/freeze_mask std_msgs/msg/UInt8 '{data: 3}'  # both
ros2 topic pub --once /io_teleop/freeze_mask std_msgs/msg/UInt8 '{data: 0}'  # resume
```
