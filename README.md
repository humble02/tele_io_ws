# IO Teleoperation ROS 2 Workspace

This workspace is a standalone copy of the IO teleoperation stack. It does not
need the original `teleop_ws_io` overlay at build or run time.

## Build

Use the system ROS environment rather than conda:

```bash
cd /home/intuitionx/tele_io_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Simulation

```bash
ros2 launch io_teleop_bringup io_teleop_sim.launch.py
```

This starts the dummy robot feedback, robot model/RViz, the IO joint-state and
hand-command bridge, and the IO target-pose to Marvin SDK IK node.

The IK command node is disabled at startup. Enable it only after dummy feedback
and `/io_teleop/target_ee_poses` are present:

```bash
ros2 service call /io_marvin_teleop/enable std_srvs/srv/SetBool '{data: true}'
```

## Real Robot

Stop any process that already owns the Marvin or Wuji Hand controllers, then:

```bash
ros2 launch io_teleop_bringup io_teleop_real.launch.py robot_ip:=192.168.1.190
```

Confirm real joint feedback and a valid IO target stream before enabling motion
with the same service command shown above.

## IO Topics

- `/io_teleop/target_ee_poses`: arm Cartesian targets consumed by Marvin SDK IK
- `/io_teleop/joint_states`: combined arm/hand feedback returned to IO
- `/io_teleop/joint_cmd`: optional direct arm joint commands forwarded by bridge
- `/io_teleop/joint_cmd_finger_left`: left-hand commands forwarded by bridge
- `/io_teleop/joint_cmd_finger_right`: right-hand commands forwarded by bridge
- `/io_teleop/freeze_mask`: bit 0 freezes the left arm/hand; bit 1 freezes the right
  arm/hand; zero resumes IO joint command control

Do not publish both `/io_teleop/joint_cmd` and `/io_teleop/target_ee_poses` for
the arms at the same time; they are two command sources for the same drivers.
