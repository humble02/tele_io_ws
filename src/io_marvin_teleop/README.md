# io_marvin_teleop

Consumes `/io_teleop/target_ee_poses`, solves both arms with the Marvin SDK IK,
and publishes device-scoped joint commands. Motion is disabled at startup.

```bash
ros2 launch io_marvin_teleop io_marvin_teleop.launch.py
ros2 service call /io_marvin_teleop/enable std_srvs/srv/SetBool '{data: true}'
```

