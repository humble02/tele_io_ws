# robot_bringup

`robot_bringup` 用于按真实 driver 或 dummy driver 两种模式启动 Marvin 双臂和 Wuji 双手，
并发布整机可视化状态。

该包提供 `scripts/joint_state_aggregator.py`，这是一个 Python ROS node，安装后可执行名为
`joint_state_aggregator`。它订阅各设备自己的关节状态话题，并发布一个全局 `/joint_states`
流给 `robot_state_publisher` 使用。

默认关节列表与 `robot_description/urdf/robot.xacro` 匹配：14 个 Marvin 机械臂关节加
40 个 Wuji 手部关节。`/joint_states` 只作为整机可视化状态流；硬件 driver 应继续发布各自
设备级的话题。

默认输入：

```text
/marvin/left/joint_states
/marvin/right/joint_states
/hand_left/joint_states
/hand_right/joint_states
```

如果某个输入话题不存在，对应关节会保持为零位。当该话题稍后出现时，匹配的关节名会覆盖默认零值。

## 真实底层 bringup

启动真实 Marvin 双臂 driver、真实 Wuji 双手 driver、聚合器、整机 `robot_state_publisher`
和 RViz：

```bash
ros2 launch robot_bringup bringup_real.launch.py
```

`bringup_real.launch.py` 会 include `marvin_bringup` 和
`wujihand_bringup/wujihand_dual_driver.launch.py`。Marvin driver 默认在 `/marvin`
namespace 下启动，`arms:=both`。Wuji hand driver 会按 USB serial number 自动发现设备，
并在检测物理左右手后发布到 `/hand_left` 或 `/hand_right`。

需要切换 Marvin 控制模式时显式指定底层 launch：

```bash
ros2 launch robot_bringup bringup_real.launch.py \
  marvin_launch:=marvin_impedance.launch.py
```

## dummy 底层 bringup

启动假的双臂双手底层 driver，以及同一套聚合器和整机可视化：

```bash
ros2 launch robot_bringup bringup_dummy.launch.py
```

`dummy_driver` 订阅正常的命令话题，并把收到的目标位置立刻发布成设备级反馈：

```text
/marvin/left/joint_commands   -> /marvin/left/joint_states
/marvin/right/joint_commands  -> /marvin/right/joint_states
/hand_left/joint_commands     -> /hand_left/joint_states
/hand_right/joint_commands    -> /hand_right/joint_states
```

这样真实遥操作输入设备和映射节点可以在没有臂/手硬件时驱动 RViz 里的整机模型。

## IO 遥操作关节桥接

启动 dummy 或实机 Marvin bringup 后，单独启动 IO 兼容桥：

```bash
ros2 launch robot_bringup io_joint_state_bridge.launch.py
```

该节点把 Marvin 机械臂反馈和 Wuji 手部反馈合成一个 54 关节
`sensor_msgs/msg/JointState`：

```text
/marvin/left/joint_states + /marvin/right/joint_states
  + /hand_left/joint_states + /hand_right/joint_states
  -> /io_teleop/joint_states
```

同时转发双臂和双手的 `sensor_msgs/msg/JointState` 命令：

```text
/io_teleop/joint_cmd -> /marvin/left/joint_commands + /marvin/right/joint_commands
/io_teleop/joint_cmd_finger_left -> /hand_left/joint_commands
/io_teleop/joint_cmd_finger_right -> /hand_right/joint_commands
```

默认 IO state 顺序为 `Joint1_R` 到 `Joint7_R`、`Joint1_L` 到 `Joint7_L`，
然后 20 个左手关节和 20 个右手关节。如果输入的 `JointState.name` 字段非空，
桥接节点会按关节名称映射命令。没有 `name` 字段时，双臂按
`io_command_joint_names` 参数定义的位置顺序解析，手部按对应手的 20 个关节默认
顺序解析。

## Marvin 曲肘动作

从 Marvin 当前实测反馈平滑移动到曲肘状态：

```bash
ros2 launch robot_bringup marvin_elbow_pose.launch.py
```

默认目标单位是度：

```text
left:  [-90,  90, 90, -90, 0, 0, 0]
right: [-90, -90, 90,  90, 0, 0, 0]
```

该节点会先等待 `/marvin/left/joint_states` 和 `/marvin/right/joint_states`，
记录当前关节角，再向 `/marvin/left/joint_commands` 和
`/marvin/right/joint_commands` 发布平滑插值后的目标。只移动单侧时可以传
`arms:=left` 或 `arms:=right`。

不打开 RViz：

```bash
ros2 launch robot_bringup bringup_dummy.launch.py rviz:=false
ros2 launch robot_bringup bringup_real.launch.py rviz:=false
```

只测真实臂、不启动真实手 driver：

```bash
ros2 launch robot_bringup bringup_real.launch.py hands:=false
```

## 运行注意

- `robot_bringup` 不启动遥操作输入包或算法包。需要遥操作时，单独启动 `wuji_glove`、
  `vive_openvr`、`wujihand_teleop` 和 `vive_marvin_teleop`。
- 全局 `/joint_states` 只给 `robot_state_publisher` 和 RViz 使用。底层 driver 和遥操作算法
  应使用 `/marvin/right/joint_states`、`/hand_right/joint_commands` 这类设备级话题。
- dummy bringup 会把命令话题直接镜像成反馈话题，适合做可视化和话题链路检查，但不能验证真实
  电机模式、控制器限位或硬件时序。
- 真实 bringup 默认 Marvin 位置模式。Vive 机械臂遥操作当前优先传入
  `marvin_launch:=marvin_impedance.launch.py`。
- `marvin_elbow_pose` 会发布机械臂运动命令。先启动正确的 dummy 或真实 Marvin bringup，
  确认反馈存在；除非是在测试仲裁，否则不要同时运行其他 Marvin command publisher。
- 真实硬件只测机械臂时，使用 `hands:=false` 避免启动 Wuji hand driver。只测灵巧手时，保持
  机械臂遥操作 disabled，并忽略 `/marvin/*` 命令话题。
- 如果 RViz 不动，先查各设备反馈，再查 `/joint_states`。缺失设备反馈时，组合模型对应部分会
  保持零位。
