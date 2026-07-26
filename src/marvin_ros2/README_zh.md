# marvin_ros2

`src/marvin_ros2` 包含当前工作区使用的 Marvin 双臂 ROS 2 集成。它是一个容器目录，
下面分成底层 driver 包和 bringup/人工工具包。

English: [README.md](README.md).

## 功能包

- [`marvin_driver`](marvin_driver/README_zh.md)：旧版 Tianji Marvin 控制器 SDK 的
  C++ ROS 2 封装。它持有控制器 TCP 连接，链接 vendored SDK，发布机械臂反馈，接收关节
  命令话题，并暴露 connect/release/estop 服务。
- [`marvin_bringup`](marvin_bringup/README_zh.md)：用于启动 driver 的 launch 文件、
  运行时 YAML、回零辅助节点和手动关节控制 UI。支持位置模式、关节阻抗模式和关节阻抗
  PD 前馈模式。

## SDK 范围

当前集成针对旧版控制器 SDK：

```bash
/home/ccs/ros2/TJ_FX_ROBOT_CONTRL_SDK/contrlSDK
```

选定 SDK 的头文件和 `libMarvinSDK.so` vendored 在
`marvin_driver/vendor/contrlSDK`。这些文件应视为选定 SDK 构建产物的副本；除非有迁移说明，
否则不要直接改 vendor 文件。

## 在工作区中的角色

- Marvin 机械臂反馈保持设备级话题：

  ```text
  /marvin/left/joint_states
  /marvin/right/joint_states
  ```

- Marvin 机械臂命令话题：

  ```text
  /marvin/left/joint_commands
  /marvin/right/joint_commands
  ```

- Marvin driver 不发布全局 `/joint_states`。整机可视化由 `robot_bringup` 负责。
- 遥操作输入包和遥操作算法包与 Marvin 硬件 bringup 分开启动。

## 快速入口

编译 Marvin 相关包：

```bash
cd /home/ccs/ros2/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select marvin_driver marvin_bringup
source install/setup.bash
```

用当前偏好的遥操作模式启动真实 driver：

```bash
ros2 launch marvin_bringup marvin_impedance.launch.py
```

直接 SDK 检查节点只能在 `marvin_driver_node` 停止时运行：

```bash
ros2 run marvin_driver marvin_link_check_node --ros-args -p robot_ip:=192.168.1.190
```

详细说明见：

- driver 内部实现和 SDK 行为：
  [`marvin_driver/README_zh.md`](marvin_driver/README_zh.md)
- launch、回零、手动 UI 和实机启动流程：
  [`marvin_bringup/README_zh.md`](marvin_bringup/README_zh.md)
