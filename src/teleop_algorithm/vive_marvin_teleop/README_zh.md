# vive_marvin_teleop

`vive_marvin_teleop` 把 Vive chest/wrist tracker TF 映射为 Marvin 机械臂 TCP
目标，使用 vendored Marvin 运动学 SDK 解 IK，并发布单臂关节命令。

这个包是遥操作算法包。它不连接 SteamVR，也不直接连接 Marvin 控制器。

## IO 末端位姿控制

包内还提供 `io_marvin_teleop`，它不使用 IO 返回的关节角，而是订阅：

```text
/io_teleop/target_ee_poses  geometry_msgs/msg/PoseArray
```

每帧的两个 pose 直接作为左右 Tianji SDK TCP 目标，经过 vendored Marvin IK 后发布到
`/marvin/left/joint_commands` 和 `/marvin/right/joint_commands`。

当前 IO 顺序是 `poses[0]` 右臂、`poses[1]` 左臂，且每个位姿需要表达在对应机械臂的
公共上身坐标系中，位置单位为米。节点使用整机 URDF 的左右臂安装变换将目标转换到
各自 Tianji SDK 基坐标系，并用 `position_scale: 0.8` 将人体/源机器人工作空间缩放到
Marvin 臂长。左右顺序、安装变换和尺度均在 `config/io_marvin_teleop.yaml` 中配置。

```bash
ros2 launch vive_marvin_teleop io_marvin_teleop.launch.py
ros2 service call /io_marvin_teleop/set_enabled std_srvs/srv/SetBool "{data: true}"
```

节点默认禁用。Tianji IK 使用 `marvin_elbow_pose_node` 的左右曲肘目标作为固定参考角，
机器人反馈仅用于反馈超时保护和首次命令衔接，不使用 `/io_teleop/joint_cmd`。
不要同时启用其他 Marvin command publisher。

## tracker 角色

当前三 tracker 配置：

```yaml
left_wrist:  "LHR-E651F39A"
right_wrist: "LHR-F757F16E"
chest:       "LHR-F854821A"
```

`vive_openvr` 会把它们发布为 `vive_world` 下的 TF：

```text
vive_world -> vive/chest
vive_world -> vive/left_wrist
vive_world -> vive/right_wrist
```

## 数据流

```text
vive_openvr TF
  -> chest 和 wrist 静态对齐 TF
  -> left_chest/right_chest 参考坐标系
  -> tianji_left/tianji_right SDK TCP 目标坐标系
  -> Marvin IK
  -> /marvin/left/joint_commands
  -> /marvin/right/joint_commands
```

默认命令输出：

```text
/marvin/left/joint_commands
/marvin/right/joint_commands
```

默认反馈输入：

```text
/marvin/left/joint_states
/marvin/right/joint_states
```

节点启动后默认禁用，不会发布命令，需要手动 enable。

## 坐标系

`vive_openvr` 的 tracker pose 全部位于 SteamVR standing space。这个包不把
`vive_world` 当机器人基座，而是用胸口 tracker 作为人体参考，再通过静态 TF 做对齐：

```text
vive/chest -> left_chest_base -> left_chest
vive/chest -> right_chest_base -> right_chest
vive/left_wrist -> tianji_left
vive/right_wrist -> tianji_right
```

含义：

- `vive/chest`：经过 `vive_openvr` 角色修正后的胸口 tracker frame。
- `left_chest_base` 和 `right_chest_base`：每侧机械臂自己的胸口安装偏移和旋转，用来把人体胸口 frame 对齐到 Marvin IK 约定。
- `left_chest` 和 `right_chest`：每条臂解 IK 时使用的参考 frame。
- `vive/left_wrist` 和 `vive/right_wrist`：经过角色修正后的手腕 tracker frame。
- `tianji_left` 和 `tianji_right`：送入 Marvin SDK IK 的目标 TCP frame。

实际控制使用的变换是：

```text
left_chest  -> tianji_left
right_chest -> tianji_right
```

Marvin IK 目标是官方 Tianji arm 代码路径使用的 SDK TCP/法兰约定，不是 Wuji 手掌
URDF link。如果 tracker 安装方向或期望工具点不对，优先改
`config/static_transforms.yaml`，尤其是 `wrist_to_tianji`，不要在控制循环里临时手写旋转。

## 启动顺序

先启动机器人侧 bringup。没有硬件时推荐先跑 dummy：

```bash
cd /home/ccs/ros2/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup bringup_dummy.launch.py
```

真实 Marvin 硬件实时遥操作当前优先使用关节阻抗模式：

```bash
ros2 launch robot_bringup bringup_real.launch.py \
  marvin_launch:=marvin_impedance.launch.py
```

启动 Vive 输入：

```bash
ros2 launch vive_openvr vive_openvr.launch.py rviz:=true
```

启动机械臂遥操作：

```bash
ros2 launch vive_marvin_teleop vive_marvin_teleop.launch.py
```

确认反馈和 TF 都正常后再打开命令输出：

```bash
ros2 service call /vive_marvin_teleop/set_enabled std_srvs/srv/SetBool "{data: true}"
```

关闭命令输出：

```bash
ros2 service call /vive_marvin_teleop/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## 检查

tracker 数据：

```bash
ros2 run vive_openvr list_trackers --all
ros2 topic echo --once /vive/chest/pose
ros2 topic echo --once /vive/left_wrist/pose
ros2 topic echo --once /vive/right_wrist/pose
```

TF 链：

```bash
ros2 run tf2_ros tf2_echo vive/chest right_chest
ros2 run tf2_ros tf2_echo left_chest tianji_left
ros2 run tf2_ros tf2_echo right_chest tianji_right
```

Marvin 反馈和命令：

```bash
ros2 topic hz /marvin/right/joint_states
ros2 topic hz /marvin/right/joint_commands
ros2 topic echo /marvin/right/joint_commands
```

如果三个 tracker pose topic 都有数据但机械臂 command topic 没数据，按顺序查：

1. 是否已经调用 `/vive_marvin_teleop/set_enabled`。
2. `/marvin/left/joint_states` 和 `/marvin/right/joint_states` 是否都有新鲜反馈。
3. `left_chest -> tianji_left` 和 `right_chest -> tianji_right` TF 是否都存在。
4. IK 是否报告目标超范围或关节限位。

## 配置文件

- `config/vive_marvin_teleop.yaml`：控制频率、反馈话题、命令话题、TF 名称、IK 分支参数、奇异容差和关节步长限制。
- `config/static_transforms.yaml`：胸口两侧对齐和手腕 tracker 到 Tianji TCP 的变换。
- `config/ccs_m6.MvKDCfg`：Marvin 运动学 SDK 配置。

`vive_marvin_teleop.yaml` 常用参数：

```text
control_rate_hz: 60.0
auto_enable: false
feedback_timeout_sec: 0.5
tf_timeout_sec: 0.5
max_joint_step_rad: 0.02
command_publish_on_change_only: false
```

`max_joint_step_rad` 限制每个控制周期发布的单关节最大变化量。早期做坐标对齐时建议保持保守。

## tracker 完整性

正常配置使用 `chest`、`left_wrist` 和 `right_wrist` 三个 tracker。完整机械臂遥操作时，
缺少任意 tracker TF 都应视为故障，而不是预期运行模式。

当前节点按双臂初始化，enable 时会检查左右两臂都有新鲜 feedback。如果 enable 后某一侧手腕
TF 缺失，该侧机械臂会跳过命令并输出限频 TF warning。真实硬件继续遥操作前，应先恢复缺失的
tracker 位姿。

## 安全注意

- 节点默认禁用。真实硬件保持 `auto_enable: false`。
- enable 前一定先确认 `/marvin/*/joint_states` 存在。
- 停止 Marvin driver 前，先通过 `/vive_marvin_teleop/set_enabled` 关闭命令输出。
- 实机运动前先在 RViz 里验证 TF。胸口旋转或 `wrist_to_tianji` 错误会产生很大的异常笛卡尔目标。
- 第一版现场对齐大概率需要调 `static_transforms.yaml` 里的旋转。把修正放在静态 TF 层，保持 IK 节点接口稳定。
