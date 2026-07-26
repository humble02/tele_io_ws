# marvin_driver

`marvin_driver` 是旧版 Tianji Marvin 控制器 SDK 的 ROS 2 C++ 封装。
它把重新编译后的旧版 SDK 共享库和头文件放在 `vendor/contrlSDK` 下。

## SDK 迁移说明

原始 SDK demo 都是直接连接控制器的独立进程。ROS driver 把这个直接 SDK 连接所有权集中在
一个节点内部，并通过 ROS 话题和服务暴露常规操作接口。

当前包中已映射：

- `showcase_new_control_sdk_usage.cpp`
  - `Connect` -> driver 启动或 `/marvin/connect`
  - `SetJointMode` -> `control_mode:=position`
  - `SetImpJointMode` -> `control_mode:=joint_impedance`
  - `SetJointPostionCmd` -> `/marvin/left/joint_commands` 和 `/marvin/right/joint_commands`
- `showcase_link_check.cpp`
  - `marvin_link_check_node`
- `showcase_cmd_delay.cpp`
  - `marvin_cmd_delay_check_node`

暂未迁移：

- PVT 和 PLN 轨迹 demo
- 拖动示教 demo
- 笛卡尔阻抗和力控 demo
- 末端 CAN/485 demo

第一版 driver 的目标是遥操作和基础设备检查，因此暂时没有加入这些功能。

`marvin_joint_control_ui.py` 这类人工操作工具放在 `marvin_bringup`，因为它只通过 ROS
话题交互，不链接 SDK。

## 话题布局

launch 文件默认把节点放在 `marvin` namespace 下。

反馈话题：

```text
/marvin/left/joint_states
/marvin/right/joint_states
```

命令话题：

```text
/marvin/left/joint_commands
/marvin/right/joint_commands
```

命令和反馈都使用 `sensor_msgs/msg/JointState`。ROS 侧位置和速度单位是弧度/弧度每秒，
driver 内部会和 SDK 的角度制 API 互相转换。

driver 有意不发布全局 `/joint_states`。全局状态流由 `robot_bringup` 里的整机聚合器负责。

## 关节名

默认关节名和 Marvin URDF 一致：

```text
Joint1_L ... Joint7_L
Joint1_R ... Joint7_R
```

如果命令消息的 `name` 字段为空，前 7 个 position 会按上述顺序解释。如果带有 name，
则必须包含该臂配置中的全部关节名。

## 控制模式

driver 支持三种 launch-time 模式：

```text
position
joint_impedance
joint_impedance_pd
```

位置模式调用：

```cpp
SetJointMode(arm, velocity_ratio, acceleration_ratio)
```

关节阻抗模式调用：

```cpp
OnSetIntPara("R.A0.BASIC.JointPIDCtlType", 0)  // 左臂
OnSetIntPara("R.A1.BASIC.JointPIDCtlType", 0)  // 右臂
SetImpJointMode(arm, velocity_ratio, acceleration_ratio, K, D)
```

关节阻抗 PD 前馈模式使用相同的阻抗设置，但在进入关节阻抗模式前，把所选臂的
`JointPIDCtlType` 设置为 `1`。

driver 不调用 `OnSavePara()`。`JointPIDCtlType` 只在运行时设置，不持久化。

三种模式启动后使用同一个关节命令 API：

```cpp
SetJointPostionCmd(arm, joint_degrees)
```

ROS 命令保持弧度制，并在 driver 内部转换。

## 启动错误处理

driver 使用 SDK 的简明 `Connect` 函数。旧 SDK 中，`Connect` 内部会调用
`CheckArmError()` 和 `CheckServoError()`。`SetJointMode` 和 `SetImpJointMode`
在模式切换前也会重复这些检查。

这意味着启动时 driver 会通过 SDK 尝试清除 arm 和 servo 错误，然后才开始接受关节命令。
如果错误清除后仍然存在，模式配置会失败，该臂会拒绝命令。

## 命令行为

driver 启动时不会发送任何默认关节命令。它只会：

- 连接控制器
- 配置请求的控制模式
- 发布反馈
- 等待 `/marvin/*/joint_commands`

需要下发运动的节点应先等待 `/marvin/*/joint_states`，并用第一帧反馈位置作为初始命令。
`marvin_zero_position_node` 遵守这个规则。

## 检查节点

这些节点会直接连接 SDK，因此只能在 `marvin_driver_node` 没有运行时使用：

```bash
ros2 run marvin_driver marvin_link_check_node --ros-args -p robot_ip:=192.168.1.190
ros2 run marvin_driver marvin_cmd_delay_check_node --ros-args -p robot_ip:=192.168.1.190 -p arm:=A
```

## 服务

默认 namespace 下：

```text
/marvin/connect
/marvin/release
/marvin/estop
/marvin/left/disable
/marvin/right/disable
```

所有服务当前都使用 `std_srvs/srv/Trigger`。

## 遥操作注意

- Vive 机械臂遥操作当前优先使用 `marvin_impedance.launch.py` bringup 路径，让命令流运行在
  显式的关节阻抗模式下。
- SDK 连接保持独占。`marvin_driver_node` 已连接时，不要同时运行 MarvinPlatform、SDK demo、
  check node 或第二个 `marvin_driver_node`。
- driver 反馈发布到 `/marvin/left/joint_states` 和 `/marvin/right/joint_states`；全局
  `/joint_states` 不由这个 driver 负责。
- 遥操作算法必须先等待新鲜反馈，并用实测机械臂状态初始化第一帧命令。在没有反馈时发布目标是
  控制层错误。
