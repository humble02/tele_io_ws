# io_joint_state_bridge

Publishes combined robot feedback on `/io_teleop/joint_states` and forwards IO
arm and hand commands to the matching Marvin and Wuji Hand device topics.

```bash
ros2 launch io_joint_state_bridge io_joint_state_bridge.launch.py
```

