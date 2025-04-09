# Pick and Place Demo

## Edge detection approach

### 1. Launch the RealSense Camera

```bash
ros2 launch realsense2_camera rs_launch.py
```

Make sure it publishes:
- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- TF frames (e.g., `camera_link`, `camera_color_optical_frame`)

### 2. Run the Shape Detection Node

```bash
ros2 run package_name sedge_detection_node
```

### 3. (Optional) Publish Static Transform

If your camera is rigidly mounted on a robot:

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 1.0 0 0 0 \
  base_link camera_link
```

Replace translation/rotation as needed.

---

### 4. 📊 Visualize in RViz2

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
