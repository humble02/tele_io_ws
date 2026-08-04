# IO 遥操作启动说明

本文说明 IO 遥操作的两种机械臂控制方式：

1. 直接使用 `/io_teleop/joint_cmd` 下发关节角。
2. 使用 `/io_teleop/target_ee_poses` 下发末端位姿，再通过天机 SDK 进行逆运动学解算。

两种方式最终都会向 Marvin 驱动的关节命令话题发送命令，因此不要同时使用两种方式控制机械臂。

## 1. 环境准备

每个终端在运行 ROS 2 命令前都需要执行：

```bash
  cd ~/tele_io_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
```

如果尚未构建工作区，先执行：

```bash
cd ~/tele_io_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. 直接使用 joint_cmd

### 2.1 仿真启动

终端 1 启动 dummy 机器人、机器人模型和 RViz：

```bash
ros2 launch robot_bringup bringup_dummy.launch.py
```

终端 2 使机械臂恢复默认姿态：

```bash
ros2 launch robot_bringup marvin_elbow_pose.launch.py
```

终端 3 启动 IO bridge：

```bash
ros2 launch io_joint_state_bridge io_joint_state_bridge.launch.py     publish_rate_hz:=50.0
```

此时 IO 端可以发布 `/io_teleop/joint_cmd`。bridge 会将左右机械臂命令拆分后发送给 dummy driver。

### 2.2 真机启动

终端 1 启动真实机器人驱动：

```bash
ros2 launch robot_bringup bringup_real.launch.py \
  robot_ip:=192.168.1.190
```

终端 2 使机械臂恢复默认姿态：

```bash
ros2 launch robot_bringup marvin_elbow_pose.launch.py
```

终端 3 启动 IO bridge：

```bash
ros2 launch io_joint_state_bridge io_joint_state_bridge.launch.py
```

```text
/io_teleop/joint_states
```

清错：

```bash
ros2 run marvin_driver marvin_link_check_node --ros-args \
  -p robot_ip:=192.168.1.190 \
  -p clear_errors:=true \
  -p frame_samples:=20 \
  -p sample_period_ms:=20
```

## 3. 使用 target_ee_poses 和天机 SDK IK

### 3.1 仿真一键启动

```bash
ros2 launch io_teleop_bringup io_teleop_sim.launch.py
```

仿真默认使用 `target_pose` 模式，并将 dummy 两臂初始反馈设置成 elbow pose。等价的完整写法是：

```bash
ros2 launch io_teleop_bringup io_teleop_sim.launch.py \
  arm_command_mode:=target_pose initial_arm_pose:=elbow
```

如果要直接使用 `/io_teleop/joint_cmd`，使用：

```bash
ros2 launch io_teleop_bringup io_teleop_sim.launch.py \
  arm_command_mode:=joint_cmd initial_arm_pose:=elbow
```

`joint_cmd` 模式会持续接受外部 PID 命令，因此初始 elbow pose 可能马上被第一帧 `/io_teleop/joint_cmd` 覆盖，这是直接关节控制的正常结果。

该 launch 会启动：

- `dummy_driver`
- `io_joint_state_bridge`
- `io_marvin_teleop`（仅 `target_pose` 模式）
- `joint_state_aggregator`
- `robot_state_publisher`
- RViz

如果需要在 `target_pose` 模式中重新回到 elbow pose，先保持 IK 禁用，然后运行：

```bash
ros2 launch robot_bringup marvin_elbow_pose.launch.py
```

在 `joint_cmd` 模式中，外部 `robot_control_pid_node` 会持续覆盖这个脚本的输出；此时需要先停止原 PID 控制链路。

### 3.2 启用 IK 控制

`io_marvin_teleop` 启动后默认处于禁用状态，不会立即发送运动命令。

首先确认左右机械臂反馈持续更新：

```bash
ros2 topic hz /marvin/left/joint_states
ros2 topic hz /marvin/right/joint_states
```

然后确认 IO 末端位姿持续更新：

```bash
ros2 topic hz /io_teleop/target_ee_poses
ros2 topic echo /io_teleop/target_ee_poses --once
```

确认三个话题正常后启用控制：

```bash
ros2 service call /io_marvin_teleop/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

正确的服务名是 `/io_marvin_teleop/set_enabled`。

启用时，节点会检查：

- 左臂反馈是否存在且未超时。
- 右臂反馈是否存在且未超时。
- `/io_teleop/target_ee_poses` 是否存在且未超时。

如果条件不满足，服务会返回 `success: false`，节点不会发送命令。

需要停止 IK 控制时执行：

```bash
ros2 service call /io_marvin_teleop/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

### 3.3 真机一键启动

```bash
ros2 launch io_teleop_bringup io_teleop_real.launch.py \
  robot_ip:=192.168.1.190
```

启动后不要立即启用控制。先确认真机左右臂反馈和 IO 末端目标都持续更新，再调用 `/io_marvin_teleop/set_enabled`。

## 4. 两种模式的选择

| 模式 | IO 输入 | 是否使用天机 IK | 主要节点 |
| --- | --- | --- | --- |
| 直接关节角 | `/io_teleop/joint_cmd` | 否 | `io_joint_state_bridge` |
| 末端位姿 | `/io_teleop/target_ee_poses` | 是 | `io_marvin_teleop` |

直接关节角模式下，不需要启动 `io_marvin_teleop`。

末端位姿模式下，可以继续运行 `io_joint_state_bridge`，用于返回 `/io_teleop/joint_states` 和转发手部命令；但 IO 端不要再向 `/io_teleop/joint_cmd` 发布机械臂命令。

## 5. 常用检查命令

查看 IO 相关节点：

```bash
ros2 node list | grep io
```

查看 IO 相关话题：

```bash
ros2 topic list | grep io_teleop
```

查看机械臂命令发布者数量：

```bash
ros2 topic info /marvin/left/joint_commands --verbose
ros2 topic info /marvin/right/joint_commands --verbose
```

正常情况下，每个机械臂命令话题只能有当前所选控制链路对应的一个有效命令发布者。
