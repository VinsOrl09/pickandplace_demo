# Pick and Place Demo

## 📦 Requirements

- ROS 2 Humble or later
- Python 3.8+
- RealSense camera (D435, D455, etc.)
- Packages:
  - `cv_bridge`
  - `tf2_ros`
  - `tf2_geometry_msgs`
  - `sensor_msgs`, `geometry_msgs`, `visualization_msgs`

Install any missing dependencies:

```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge \
                 ros-${ROS_DISTRO}-tf2-ros \
                 ros-${ROS_DISTRO}-tf2-geometry-msgs \
                 ros-${ROS_DISTRO}-vision-msgs
```

---

## 🚀 How to Run

### 1. Clone and Build

```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Launch the RealSense Camera

```bash
ros2 launch realsense2_camera rs_launch.py
```


### 3. Run the Shape Detection Node

```bash
ros2 run shape_detector shape_detector_node
```

### 4. (Optional) Publish Static Transform

If your camera is rigidly mounted on a robot:

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 1.0 0 0 0 \
  base_link camera_link
```

Replace translation/rotation as needed.

---

### 5. 📊 Visualize in RViz2

```bash
rviz2
```

Add these displays:
- **Image**:
  - Topic: `/camera/color/image_raw`
- **Image**:
  - Topic: `/edges`
- **MarkerArray**:
  - Topic: `/shape_markers`
- **TF**: to visualize frames

---

