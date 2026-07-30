# io_joint_state_bridge

Publishes Marvin arm feedback on `/io_teleop/joint_states` and forwards IO arm
and hand commands to the matching Marvin and Wuji Hand device topics. Hand state
feedback is not subscribed to or included in the IO state topic.

```bash
ros2 launch io_joint_state_bridge io_joint_state_bridge.launch.py
```
