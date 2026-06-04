# aruco_depth_ros2

---

## Run

### Run node directly

```bash
ros2 run aruco_depth_ros2 aruco_depth_node
```

### Run with explicit parameters

```bash
ros2 run aruco_depth_ros2 aruco_depth_node --ros-args \
  -p color_topic:=/j100_0897/sensors/camera_0/color/image \
  -p depth_topic:=/j100_0897/sensors/camera_0/depth/image \
  -p camera_info_topic:=/j100_0897/sensors/camera_0/color/camera_info \
  -p marker_size:=0.07 \
  -p aruco_dictionary:=25h9 \
  -p target_id:=-1 \
  -p arm_base_frame:=arm_0_base_link \
  -p world_frame:=base_link \
  -p publish_tf:=true \
  -p publish_rviz_markers:=false \
  -p show_window:=false
```

### Run launch file

```bash
ros2 launch aruco_depth_ros2 aruco_realsense.launch.py
```

---

## Parameters

### Camera + calibration

| Parameter | Type | Default | Description |
|---|---:|---|---|
| `color_topic` | string | `/j100_0897/sensors/camera_0/color/image` | RGB/color image topic used for ArUco detection. |
| `depth_topic` | string | `/j100_0897/sensors/camera_0/depth/image` | Depth image topic used to estimate marker center depth. |
| `camera_info_topic` | string | `/j100_0897/sensors/camera_0/color/camera_info` | Camera calibration topic used for pose estimation. |

### ArUco detection

| Parameter | Type | Default | Description |
|---|---:|---|---|
| `marker_size` | double | `0.05` | Physical side length of the ArUco marker in meters. `0.05` means 50 mm. |
| `aruco_dictionary` | string | `original` | ArUco dictionary used for detection. |
| `target_id` | int | `-1` | `-1` processes all markers. Any other value processes only that marker ID. |

Supported `aruco_dictionary` values:

```text
original
4x4_50, 4x4_100, 4x4_250, 4x4_1000
5x5_50, 5x5_100, 5x5_250, 5x5_1000
6x6_50, 6x6_100, 6x6_250, 6x6_1000
7x7_50, 7x7_100, 7x7_250, 7x7_1000
```

### Outputs + visualization

| Parameter | Type | Default | Description |
|---|---:|---|---|
| `publish_tf` | bool | `true` | Broadcasts a TF frame for each detected marker. |
| `publish_rviz_markers` | bool | `true` | Publishes RViz cube markers for each detected marker. |
| `show_window` | bool | `false` | Opens an OpenCV window showing the debug image. Usually keep this `false` on a robot or headless system. |

### Frame transforms

| Parameter | Type | Default | Description |
|---|---:|---|---|
| `arm_base_frame` | string | `arm_0_base_link` | Target frame for arm-relative marker poses. |
| `world_frame` | string | `base_link` | Target frame for robot/world-relative marker poses. |

---

## Published Topics

### `/aruco/debug_image`

Message type:

```bash
sensor_msgs/msg/Image
```

Publishes the camera image with ArUco detection overlays.

When markers are detected, the image includes:

- Marker outline.
- Marker ID.
- Marker center point.
- Estimated depth at the marker center.
- Estimated pose position.
- Coordinate axes drawn on top of the marker.

Example overlay text:

```text
ID: 5
Depth: 0.842 m
Pose: x=0.12, y=-0.03, z=0.85
```

View it with:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select:

```text
/aruco/debug_image
```

---

### `/aruco/pose`

Message type:

```bash
geometry_msgs/msg/PoseStamped
```

Publishes the pose of each detected marker in the camera frame.

This topic is useful for quick single-marker testing.

Important: this topic does **not** include the marker ID. If multiple markers are detected, the node publishes multiple `PoseStamped` messages one after another. For multiple-marker detection, use `/aruco/marker_ids` together with one of the pose array topics.

Echo:

```bash
ros2 topic echo /aruco/pose
```

---

### `/aruco/poses/camera_frame`

Message type:

```bash
geometry_msgs/msg/PoseArray
```

Publishes all detected marker poses relative to the camera frame.

The pose position comes from OpenCV `solvePnP`:

- `x`: marker position left/right relative to the camera.
- `y`: marker position up/down relative to the camera.
- `z`: marker distance forward from the camera.

Echo:

```bash
ros2 topic echo /aruco/poses/camera_frame
```

---

### `/aruco/poses/arm_base_frame`

Message type:

```bash
geometry_msgs/msg/PoseArray
```

Publishes all detected marker poses transformed into the frame specified by `arm_base_frame`.

Default target frame:

```text
arm_0_base_link
```

Use this output when the robot arm needs marker poses relative to the arm base.

This topic requires a valid TF transform from the camera frame to `arm_0_base_link`. If the transform is missing, this topic may publish an empty pose array.

Echo:

```bash
ros2 topic echo /aruco/poses/arm_base_frame
```

---

### `/aruco/poses/world_frame`

Message type:

```bash
geometry_msgs/msg/PoseArray
```

Publishes all detected marker poses transformed into the frame specified by `world_frame`.

Default target frame:

```text
base_link
```

Use this output when marker locations are needed relative to the robot base or world reference frame.

This topic requires a valid TF transform from the camera frame to `base_link`. If the transform is missing, this topic may publish an empty pose array.

Echo:

```bash
ros2 topic echo /aruco/poses/world_frame
```

---

### `/aruco/marker_ids`

Message type:

```bash
std_msgs/msg/Int32MultiArray
```

Publishes the IDs of detected markers.

The ID order matches the pose order in the pose arrays.

Example:

```text
/aruco/marker_ids:
data: [3, 7, 12]
```

Then the matching pose order is:

```text
/aruco/poses/camera_frame.poses[0] -> marker ID 3
/aruco/poses/camera_frame.poses[1] -> marker ID 7
/aruco/poses/camera_frame.poses[2] -> marker ID 12
```

This is the recommended way to associate marker IDs with poses when detecting multiple markers.

Echo:

```bash
ros2 topic echo /aruco/marker_ids
```

---

### `/aruco/marker_array`

Message type:

```bash
visualization_msgs/msg/MarkerArray
```

Publishes green cube markers for RViz. Each cube represents a detected ArUco marker pose.

The cube size is based on the `marker_size` parameter. The marker is thin in the z-direction so it appears like a flat square.

In RViz:

1. Add a `MarkerArray` display.
2. Set the topic to:

```text
/aruco/marker_array
```

---

## TF Output

If `publish_tf` is enabled, the node broadcasts a TF frame for each detected marker.

Each marker frame is named:

```text
aruco_marker_<id>
```

Examples:

```text
aruco_marker_0
aruco_marker_5
aruco_marker_23
```

The parent frame is the frame from the incoming RGB image header.

Example TF relationship:

```text
camera_frame
└── aruco_marker_5
```

Inspect the TF tree:

```bash
ros2 run tf2_tools view_frames
```

View a specific transform:

```bash
ros2 run tf2_ros tf2_echo <camera_frame> aruco_marker_5
```

Replace `<camera_frame>` with the actual camera frame used by the camera image topic, for example:

```bash
camera_0_color_optical_frame
```

---

## Terminal Output

When the node starts, it prints the configured topics, frames, and parameters.

Example startup log:

```text
ArUco depth ROS 2 node started.
RGB topic: /j100_0897/sensors/camera_0/color/image
Depth topic: /j100_0897/sensors/camera_0/depth/image
Camera info topic: /j100_0897/sensors/camera_0/color/camera_info
Marker size: 0.05 m
Dictionary: original
Target ID: -1  (-1 means all markers)
Arm base frame: arm_0_base_link
World frame: base_link
Publish TF: True
Publish RViz markers: True
```

When a marker is detected, it logs the marker ID, depth, pose, and frame.

Example detection log:

```text
Marker ID: 5 | Depth: 0.842 m | Pose: x=0.120, y=-0.034, z=0.847 m | Frame: camera_0_color_optical_frame
```

---

## Recommended Usage

### Single-marker debugging

Use:

```bash
ros2 topic echo /aruco/pose
ros2 run rqt_image_view rqt_image_view
```

Then view:

```text
/aruco/debug_image
```

### Multiple-marker detection

Use:

```bash
ros2 topic echo /aruco/marker_ids
ros2 topic echo /aruco/poses/camera_frame
```

Match marker IDs to poses by index.

Example:

```text
marker_ids.data[0] corresponds to poses[0]
marker_ids.data[1] corresponds to poses[1]
marker_ids.data[2] corresponds to poses[2]
```

### Robot arm manipulation

Use:

```bash
ros2 topic echo /aruco/poses/arm_base_frame
```


### Robot/world-relative localization

Use:

```bash
ros2 topic echo /aruco/poses/world_frame
```

