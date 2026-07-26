# IO Mocap Visualization

`io_mocap_visualization` is a ROS 2 package designed to visualize human motion capture data. It bridges external Motion Capture (MoCap) data published on custom topics (e.g., `/io_fusion/tf`) to standard ROS 2 TF frames, enabling real-time or playback visualization in RViz2 using custom URDF models.

## Getting Started

### 1. Prerequisites

- ROS 2 Distribution: Humble or newer.
- Dependencies:

```
sudo apt install ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-joint-state-publisher-gui \
                 ros-$ROS_DISTRO-rviz2
```

### 2. Installation

Clone this repository into the src folder of your ROS 2 workspace:

```
cd ~/ros2_ws/src
git clone git@github.com:ioai-tech/io_mocap_visualization.git
```

### 3. Build the Workspace

Navigate back to the workspace root and build using `colcon`:

```
cd ~/ros2_ws
colcon build --packages-select io_mocap_visualization
source install/setup.bash
```

### 4. Launch Visualization

Start the visualization node, which initializes the `robot_state_publisher`, the TF relay, and RViz2:

```
ros2 launch io_mocap_visualization io_mocap_visualization.launch.py
```

### 5. Playback Data

To visualize motion from a recorded ROS 2 bag file containing the /io_fusion/tf topic, use the --clock flag to ensure time synchronization:

```
ros2 bag play <your_ros2_bag_path> --clock
```

## System Architecture

The package functions by re-routing specific mocap data to the standard ROS 2 visualization pipeline:

Input: ROS 2 Bag sends `/io_fusion/tf`(tf2_msgs/msg/TFMessage).

Relay: The `tf_relay` node listens to `/io_fusion/tf` and publishes to `/tf`.

Model: `robot_state_publisher` loads the URDF and maintains the model structure.

Display: RViz2 renders the `blender_human` model based on the calculated transforms.
