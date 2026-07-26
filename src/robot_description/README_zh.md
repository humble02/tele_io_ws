# robot_description

这是遥操作机器人的顶层描述包，负责最终组合模型，并引用资源包
`marvin_description`、`wuji_description` 和 `realsense2_description`。

## 机器人配置

组合机器人定义在 `urdf/robot.xacro` 中，并展开生成 `urdf/robot.urdf`。

`robot_description` 是当前工作区的主要描述入口。它把 `src/` 下的资源包组合成最终整机模型：

- `marvin_description`：Marvin M6-S 左右机械臂 URDF 和 mesh。
- `wuji_description`：Wuji 左右手 URDF、mesh 和手部 MJCF 资源。
- `realsense2_description`：RealSense 相机 xacro 模型和 mesh。

最终包会加入共享底座、升降平台、手部转接件、相机转接件、展开后的 URDF、RViz 配置和 MuJoCo
场景。

配置如下：

- 底座：`base_link` 车体和固定的 `lift_platform_link`。
- 机械臂：两条 Marvin M6-S 七自由度机械臂。
  - 左臂关节：`Joint1_L` 到 `Joint7_L`。
  - 右臂关节：`Joint1_R` 到 `Joint7_R`。
- 手：两只无极五指手。
  - 每只手有 20 个转动手指关节。
  - 左手关节使用 `left_finger*_joint*` 命名。
  - 右手关节使用 `right_finger*_joint*` 命名。
- 相机：一个头部 RealSense D435i 和两个腕部 RealSense D435i。

可驱动模型自由度：

- 机械臂：共 14 自由度。
- 双手：共 40 自由度。
- 总关节自由度：54。

底座、升降平台、手部对接件、腕部相机转接件和相机 frame 都是固定 link。

## 遥操作映射参考 frame

后续做遥操作映射时，可使用这些稳定 link frame：

- `base_link`：整机根车体 frame。
- `lift_platform_link`：两条机械臂安装的固定平台 frame。
- `Base_L`、`Base_R`：左右臂基座 frame。
- `Link7_L`、`Link7_R`：左右臂腕部/法兰 frame。
- `left_hand_docking_link`、`right_hand_docking_link`：从腕部安装的手部固定对接转接件。
- `left_palm_link`、`right_palm_link`：手掌 frame，可作为手部任务 frame。
- `left_camera_adapter_link`、`right_camera_adapter_link`：腕部相机转接件 frame。
- `head_camera_color_optical_frame`：头部 RGB optical frame。
- `left_wrist_camera_color_optical_frame`、`right_wrist_camera_color_optical_frame`：腕部 RGB optical frame。

ROS optical frame 使用标准相机约定：`+X` 向右、`+Y` 向下、`+Z` 向前。MuJoCo XML 中，
相机会挂在对应的 `*_color_optical_frame` 下，并旋转回 OpenGL/MuJoCo 相机约定，
即 `+Z` 指向后方。

Vive 到 Marvin 的机械臂遥操作节点不会直接把这些 URDF link 当成 IK 目标接口。
`vive_marvin_teleop` 送入 Marvin SDK IK wrapper 的是 `left_chest -> tianji_left` 和
`right_chest -> tianji_right`。这些 `tianji_*` frame 遵循 SDK TCP/法兰约定，不是
`left_palm_link` 或 `right_palm_link`。URDF link 主要用于可视化和标定参考；tracker 到
TCP 的对齐应在 `vive_marvin_teleop/config/static_transforms.yaml` 中调整。

## 视觉

头部相机是由 `realsense2_description/urdf/_d435i.urdf.xacro` 生成的 RealSense D435i。

当前头部相机安装：

- 父 link：`base_link`。
- 安装 frame：`head_camera_bottom_screw_frame`。
- 相对 `base_link` 的位置：`xyz="0.30 0.0070867 1.4"`。
- 相对 `base_link` 的姿态：`rpy="0 1.047198 0"`，即绕 Y 轴旋转 60 度。
- 主 RGB optical frame：`head_camera_color_optical_frame`。

腕部相机安装在：

- `left_camera_adapter_link`，命名为 `left_wrist_camera`。
- `right_camera_adapter_link`，命名为 `right_wrist_camera`。

## 重要文件

- `urdf/robot.xacro`：组合机器人源 xacro。
- `urdf/robot.urdf`：展开后的 URDF，供直接消费和 MuJoCo 导出使用。
- `urdf/marvin_left.xacro`、`urdf/marvin_right.xacro`：机械臂封装宏。
- `urdf/wuji_left.xacro`、`urdf/wuji_right.xacro`：手部封装宏。
- `meshes/`：底座、升降平台、手部对接件和腕部相机转接件 mesh。
- `mjcf/robot.xml`：最终 MuJoCo robot XML。
- `mjcf/robot_raw.xml`：保留用于调试的原始 MuJoCo 导出结果。
- `mjcf/_robot_mujoco_input.urdf`：MuJoCo 转换用临时 URDF。
- `mjcf/scene.xml`：带地面和桌子的 MuJoCo 场景。
- `mjcf/scene.py`：用于三个机器人相机的 OpenCV/MuJoCo 渲染器。

## 构建和运行

从工作区根目录运行：

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_description view_robot.launch.py
```

zsh 环境：

```bash
source install/setup.zsh
```

从 xacro 重新生成展开后的 URDF：

```bash
xacro src/robot_description/urdf/robot.xacro > src/robot_description/urdf/robot.urdf
```

运行 MuJoCo 场景：

```bash
conda run -n floating python src/robot_description/mjcf/scene.py
```

只渲染指定相机：

```bash
conda run -n floating python src/robot_description/mjcf/scene.py \
  --camera head_camera \
  --camera left_wrist_camera
```
