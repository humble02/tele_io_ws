# marvin_bringup

`marvin_bringup` 负责 Marvin 的 launch 文件、运行时 YAML、回零辅助节点和手动关节控制
UI。它不直接链接 Marvin SDK；真实控制器连接由 `marvin_driver` 持有。

English: [README.md](README.md).

## 构建

从工作区根目录运行：

```bash
cd /home/ccs/ros2/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select marvin_driver marvin_bringup
source install/setup.bash
```

不要在 conda 环境里编译 ROS 包。

## driver 启动 launch

启动真实硬件前，先确认控制器 IP 可达，并且没有其他 SDK 客户端正在连接。默认控制器 IP 是
`192.168.1.190`，实际不一致时用 `robot_ip:=...` 覆盖。

位置模式：

```bash
ros2 launch marvin_bringup marvin_position.launch.py robot_ip:=192.168.1.190 arms:=both
```

关节阻抗模式：

```bash
ros2 launch marvin_bringup marvin_impedance.launch.py robot_ip:=192.168.1.190 arms:=both
```

关节阻抗 PD 前馈模式：

```bash
ros2 launch marvin_bringup marvin_impedance_pd.launch.py robot_ip:=192.168.1.190 arms:=both
```

当前 Vive 机械臂遥操作测试优先使用普通关节阻抗模式：

```bash
ros2 launch robot_bringup bringup_real.launch.py \
  marvin_launch:=marvin_impedance.launch.py
```

常用 launch 参数：

```text
namespace:=marvin
robot_ip:=192.168.1.190
arms:=both | left | right
auto_connect:=true | false
velocity_ratio:=10
acceleration_ratio:=10
```

## 回零辅助节点

确认 driver 已启动、`/marvin/*/joint_states` 稳定发布后，才运行回零辅助节点：

```bash
ros2 launch marvin_bringup marvin_zero.launch.py arms:=both
```

`marvin_zero_position_node` 会先等待反馈，用当前实测关节位置初始化第一帧命令，然后插值到零位。
它不会随 driver 自动启动。

常用回零参数：

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

## 手动关节控制 UI

运行 Tk 手动 UI：

```bash
ros2 run marvin_bringup marvin_joint_control_ui.py
```

UI 订阅：

```text
/marvin/left/joint_states
/marvin/right/joint_states
```

并发布：

```text
/marvin/left/joint_commands
/marvin/right/joint_commands
```

默认情况下，UI 会在收到反馈前拒绝发布命令。如果 Tkinter import 失败，安装 `python3-tk`。

常用 UI 参数：

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

示例：

```bash
ros2 run marvin_bringup marvin_joint_control_ui.py --ros-args \
  -p arms:=right \
  -p require_feedback:=true
```

## 实机分步启动

第一次实机测试建议先单臂、慢速回零。

终端 1：只启动 Marvin driver，进入位置模式：

```bash
cd /home/ccs/ros2/teleop_ws
source install/setup.bash
ros2 launch marvin_bringup marvin_position.launch.py \
  robot_ip:=192.168.1.190 \
  arms:=left \
  auto_connect:=true
```

终端 2：确认反馈：

```bash
cd /home/ccs/ros2/teleop_ws
source install/setup.bash
ros2 topic echo --once /marvin/left/joint_states
ros2 topic hz /marvin/left/joint_states
```

终端 2：确认反馈正常后慢速回零：

```bash
ros2 launch marvin_bringup marvin_zero.launch.py \
  arms:=left \
  move_duration_sec:=15.0
```

双臂确认安全后再使用。

## 硬件注意事项

- 确认急停、使能、电源、气路和外设处于预期状态，机械臂运动范围内无人和障碍物。
- 启动 driver launch 前，确认 MarvinPlatform_EN、SDK demo、check node 和其他直接 SDK
  客户端都已退出。
- 切换位置模式、关节阻抗模式或 PD 前馈模式前，机械臂应处于静止状态。
- `joint_impedance_k` 和 `joint_impedance_d` 先在 YAML 中调试。除非明确要求，不要持久化写入控制器。
- `robot_bringup` 发布的 `/joint_states` 只用于整机显示。检查 driver 反馈时使用
  `/marvin/*/joint_states`。
