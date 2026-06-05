# Placement Accuracy Checker Robot Test Guide

This guide explains how to test the placement accuracy checker on the Clearpath robot using a hardcoded planner input.

The checker performs the following pipeline:

```text
Robot RGB-D camera
    ↓
AprilTag 16h5 detection inside placement_accuracy_checker.py
    ↓
Current block pose estimated in camera frame
    ↓
Pose transformed into target/world frame using TF
    ↓
Compared against hardcoded planner desired pose
    ↓
Returns good / misplaced / unseen
```

## Current Test Tag

```text
AprilTag family: 16h5
Tag ID: 13
Tag size: 55 mm = 0.055 m
```

## Safety Note

Correction is disabled during this test:

```bash
-p enable_correction:=false
```

The checker will only report whether the block is properly placed, misplaced, or unseen. It will not command the arm or gripper.

---

## 1. SSH into the Robot

```bash
ssh robot@192.168.0.20
```

Then source the workspace:

```bash
cd ~/aruco_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## 2. Confirm Camera Topics

```bash
ros2 topic list | grep -E "camera|color|depth|camera_info"
```

The robot may use Clearpath-style topics:

```text
/j100_0897/sensors/camera_0/color/image
/j100_0897/sensors/camera_0/depth/image
/j100_0897/sensors/camera_0/color/camera_info
```

or RealSense-style topics:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

---

## 3. Confirm Camera Frame

For Clearpath-style topics:

```bash
ros2 topic echo /j100_0897/sensors/camera_0/color/image --once | grep frame_id
```

For RealSense-style topics:

```bash
ros2 topic echo /camera/camera/color/image_raw --once | grep frame_id
```

Write down the camera frame. It may be something like:

```text
camera_color_optical_frame
```

---

## 4. Check TF Connection

If the planner/checker target frame is `world`:

```bash
ros2 run tf2_ros tf2_echo world <camera_frame>
```

If that fails, try `odom`:

```bash
ros2 run tf2_ros tf2_echo odom <camera_frame>
```

Use whichever frame works as the checker `target_frame`.

If `odom` works but `world` does not, either run the checker with:

```bash
-p target_frame:=odom
```

or temporarily publish:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id world \
  --child-frame-id odom
```

---

## 5. Run the Placement Checker

### Option A: Clearpath Camera Topics

```bash
cd ~/aruco_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run manipulator placement_accuracy_checker --ros-args \
  -p color_topic:=/j100_0897/sensors/camera_0/color/image \
  -p depth_topic:=/j100_0897/sensors/camera_0/depth/image \
  -p camera_info_topic:=/j100_0897/sensors/camera_0/color/camera_info \
  -p tag_family:=16h5 \
  -p target_id:=13 \
  -p marker_size:=0.055 \
  -p target_frame:=odom \
  -p fallback_source_frame:=camera_color_optical_frame \
  -p enable_correction:=false \
  -p theta_tolerance_deg:=180.0
```

### Option B: RealSense-Style Camera Topics

```bash
cd ~/aruco_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run manipulator placement_accuracy_checker --ros-args \
  -p color_topic:=/camera/camera/color/image_raw \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info \
  -p tag_family:=16h5 \
  -p target_id:=13 \
  -p marker_size:=0.055 \
  -p target_frame:=odom \
  -p fallback_source_frame:=camera_color_optical_frame \
  -p enable_correction:=false \
  -p theta_tolerance_deg:=180.0
```

Expected log when the tag is visible:

```text
Tag ID 13 perceived.
Camera pose: x=..., y=..., z=...
World pose: x=..., y=..., z=...
```

The log may say “World pose,” but the actual frame is the value set by `target_frame`, for example `odom`.

---

## 6. View Debug Image

```bash
ros2 run rqt_image_view rqt_image_view /placement_checker/debug_image
```

The debug image should show AprilTag ID `13` when visible.

---

## 7. Hardcoded Planner Service Call

The planner is not calling the checker yet, so manually call the service.

Always refresh before calling:

```bash
cd ~/aruco_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 daemon stop
ros2 daemon start
```

Check the service:

```bash
ros2 service list | grep placement
ros2 service type /placement_checker/check_placement
```

Expected:

```text
/placement_checker/check_placement
cc_interfaces/srv/CheckPlacement
```

---

## 8. Test A: Unseen Block

Move the tag out of camera view:

```bash
ros2 service call /placement_checker/check_placement cc_interfaces/srv/CheckPlacement "{block_ids: ['block_13'], aruco_ids: [13], desired_poses: [{header: {frame_id: 'odom'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}"
```

Expected:

```text
unseen=['block_13']
```

---

## 9. Test B: Misplaced Block

Put the tag in view and use a desired pose far away:

```bash
ros2 service call /placement_checker/check_placement cc_interfaces/srv/CheckPlacement "{block_ids: ['block_13'], aruco_ids: [13], desired_poses: [{header: {frame_id: 'odom'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}"
```

Expected:

```text
misplaced=['block_13']
```

---

## 10. Test C: Properly Placed Block

Look at the placement checker terminal and copy the current transformed pose:

```text
World pose: x=..., y=..., z=...
```

Use those values in the service call.

Example:

```bash
ros2 service call /placement_checker/check_placement cc_interfaces/srv/CheckPlacement "{block_ids: ['block_13'], aruco_ids: [13], desired_poses: [{header: {frame_id: 'odom'}, pose: {position: {x: -0.033, y: 0.115, z: 0.470}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}"
```

Expected:

```text
good=['block_13']
```

---

## 11. RViz Check

Start RViz:

```bash
rviz2
```

Set the fixed frame to:

```text
odom
```

or:

```text
world
```

depending on the checker `target_frame`.

Add these displays:

```text
TF
/placement_checker/marker_array
/placement_checker/debug_image
```

The checker marker array should show:

```text
green = properly placed
red = misplaced
```

---

## Expected Results

| Test                                  | Expected Result          |
| ------------------------------------- | ------------------------ |
| Tag out of view                       | `unseen=['block_13']`    |
| Tag visible but desired pose far away | `misplaced=['block_13']` |
| Desired pose matches perceived pose   | `good=['block_13']`      |

