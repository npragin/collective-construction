# Placement Accuracy Checker Robot Test Guide

This guide explains how to run and test the placement accuracy checker on the Clearpath J100 robot.

The checker performs the following pipeline:

```text
Robot RGB-D camera
    ↓
AprilTag 16h5 detection inside placement_accuracy_checker.py
    ↓
Current block pose estimated in camera frame
    ↓
Pose transformed into the robot world frame using TF
    ↓
Compared against planner desired pose
    ↓
Returns good / misplaced / unseen
```

## Current Tested Setup

Robot namespace:

```text
/j100_0897
```

Test tag:

```text
AprilTag family: 16h5
Tag ID: 13
Tag size: 55 mm = 0.055 m
```

Robot camera topics:

```text
Color image:
  /j100_0897/sensors/camera_0/color/image

Depth image:
  /j100_0897/sensors/camera_0/depth/image

Camera info:
  /j100_0897/sensors/camera_0/depth/camera_info
```

Robot TF topics:

```text
Dynamic TF:
  /j100_0897/tf

Static TF:
  /j100_0897/tf_static
```

Target frame:

```text
world
```

Camera frame:

```text
camera_0_color_optical_frame
```

---

## 1. SSH into the Robot

From your laptop:

```bash
ssh robot@192.168.0.20
```

Example:

```bash
ssh robot@192.168.0.20
```

Then source the robot workspace:

```bash
cd ~/ws/rob599

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## 2. Confirm the Camera Feed

Check that the robot camera topics are active:

```bash
ros2 topic info /j100_0897/sensors/camera_0/color/image -v
ros2 topic info /j100_0897/sensors/camera_0/depth/image -v
ros2 topic info /j100_0897/sensors/camera_0/depth/camera_info -v
```

The color topic should have a publisher.

Confirm the color image frame:

```bash
ros2 topic echo /j100_0897/sensors/camera_0/color/image --once | grep frame_id
```

Expected:

```text
frame_id: camera_0_color_optical_frame
```

---

## 3. Confirm the World Frame is Being Broadcast

The checker needs the robot world frame to be available in the namespaced TF tree.

Run:

```bash
ros2 topic echo /j100_0897/tf | grep -A 8 -B 2 "frame_id: world"
```

Expected output should include something like:

```text
frame_id: world
child_frame_id: odom
```

If `world -> odom` is not being broadcast, the checker will not be able to transform detected block poses into the planner’s world frame.

The expected TF chain is:

```text
world
  ↓
odom
  ↓
base_link
  ↓
camera_0_link
  ↓
camera_0_color_optical_frame
```

---

## 4. Run the Placement Checker

Run this in a new terminal on the robot:

```bash
cd ~/ws/rob599

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run manipulator placement_accuracy_checker --ros-args \
  -p color_topic:=/j100_0897/sensors/camera_0/color/image \
  -p depth_topic:=/j100_0897/sensors/camera_0/depth/image \
  -p camera_info_topic:=/j100_0897/sensors/camera_0/depth/camera_info \
  -p tag_family:=16h5 \
  -p target_id:=13 \
  -p marker_size:=0.055 \
  -p target_frame:=world \
  -p fallback_source_frame:=camera_0_color_optical_frame \
  -p tf_topic:=/j100_0897/tf \
  -p tf_static_topic:=/j100_0897/tf_static \
  -p enable_correction:=false \
  -p theta_tolerance_deg:=180.0
```

Expected startup logs should include:

```text
Placement accuracy checker with perception started.
Tag family: 16h5
Target ID: 13
Target/world frame: world
Correction enabled: False
```

When AprilTag ID 13 is visible, the checker should print:

```text
Tag ID 13 perceived.
Camera pose: x=..., y=..., z=...
World pose: x=..., y=..., z=...
```

---

## 5. View the Camera or Debug Image

If using SSH, connect with X forwarding:

```bash
ssh -Y robot@<robot_ip>
```

Then run:

```bash
cd ~/ws/rob599

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rqt_image_view rqt_image_view /j100_0897/sensors/camera_0/color/image
```

To view the checker overlay:

```bash
ros2 run rqt_image_view rqt_image_view /placement_checker/debug_image
```

If using a normal SSH terminal without X forwarding, GUI tools will fail with:

```text
could not connect to display
```

---

## 6. Hardcoded Planner Test: Misplaced Case

This simulates a planner desired pose at the world origin.

With AprilTag ID 13 visible, run:

```bash
cd ~/ws/rob599

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service call /placement_checker/check_placement cc_interfaces/srv/CheckPlacement "{block_ids: ['block_13'], aruco_ids: [13], desired_poses: [{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}"
```

Expected result if the detected block is not at the origin:

```text
misplaced=['block_13']
```

Example successful misplaced response:

```text
cc_interfaces.srv.CheckPlacement_Response(
  success=True,
  message="Placement check complete. good=[], misplaced=['block_13'], unseen=[]",
  properly_placed_ids=[],
  misplaced_ids=['block_13'],
  unseen_ids=[]
)
```

---

## 7. Hardcoded Planner Test: Properly Placed Case

Look at the checker terminal and copy the latest printed world pose.

Example:

```text
World pose: x=0.871, y=0.448, z=0.126
```

Use those values as the desired pose:

```bash
ros2 service call /placement_checker/check_placement cc_interfaces/srv/CheckPlacement "{block_ids: ['block_13'], aruco_ids: [13], desired_poses: [{header: {frame_id: 'world'}, pose: {position: {x: 0.871, y: 0.448, z: 0.126}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}"
```

Expected result:

```text
good=['block_13']
misplaced=[]
unseen=[]
```

Example successful properly placed response:

```text
cc_interfaces.srv.CheckPlacement_Response(
  success=True,
  message="Placement check complete. good=['block_13'], misplaced=[], unseen=[]",
  properly_placed_ids=['block_13'],
  misplaced_ids=[],
  unseen_ids=[]
)
```

---

## 8. Hardcoded Planner Test: Unseen Case

Move AprilTag ID 13 out of the camera view.

Then call the service again:

```bash
ros2 service call /placement_checker/check_placement cc_interfaces/srv/CheckPlacement "{block_ids: ['block_13'], aruco_ids: [13], desired_poses: [{header: {frame_id: 'world'}, pose: {position: {x: 0.871, y: 0.448, z: 0.126}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}"
```

Expected result:

```text
unseen=['block_13']
```

---

## 9. Optional: Enable Correction Action

By default, correction is disabled:

```bash
-p enable_correction:=false
```

This means the checker will only report:

```text
good / misplaced / unseen
```

To test the correction action pipeline, start the correction task server:

```bash
cd ~/ws/rob599

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run manipulator correction_task_server
```

The correction server may wait for the AbsoluteMove action server:

```text
Waiting for AbsoluteMove action server...
```

If so, start the AbsoluteMove server in another terminal:

```bash
cd ~/ws/rob599

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run manipulator absolute_move
```

Confirm the correction action exists:

```bash
ros2 action list | grep correction
ros2 action info /manipulator/correction_task
```

Expected:

```text
/manipulator/correction_task
Action servers: 1
```

Then restart the checker with correction enabled:

```bash
ros2 run manipulator placement_accuracy_checker --ros-args \
  -p color_topic:=/j100_0897/sensors/camera_0/color/image \
  -p depth_topic:=/j100_0897/sensors/camera_0/depth/image \
  -p camera_info_topic:=/j100_0897/sensors/camera_0/depth/camera_info \
  -p tag_family:=16h5 \
  -p target_id:=13 \
  -p marker_size:=0.055 \
  -p target_frame:=world \
  -p fallback_source_frame:=camera_0_color_optical_frame \
  -p tf_topic:=/j100_0897/tf \
  -p tf_static_topic:=/j100_0897/tf_static \
  -p correction_action_name:=/manipulator/correction_task \
  -p enable_correction:=true \
  -p theta_tolerance_deg:=180.0
```

When a block is classified as misplaced, the checker should send a `CorrectionTask` action goal to:

```text
/manipulator/correction_task
```

---

## 10. Troubleshooting

### Checker detects tag but cannot transform to world

Error:

```text
Could not transform from camera_0_color_optical_frame to world:
"world" passed to lookupTransform argument target_frame does not exist.
```

Fix:

Make sure `world -> odom` is being broadcast:

```bash
ros2 topic echo /j100_0897/tf | grep -A 8 -B 2 "frame_id: world"
```

Expected:

```text
frame_id: world
child_frame_id: odom
```

### Checker launches but no tag is detected

Check that the correct tag is being used:

```text
AprilTag family: 16h5
Tag ID: 13
Tag size: 0.055 m
```

Also check the camera feed:

```bash
ros2 run rqt_image_view rqt_image_view /j100_0897/sensors/camera_0/color/image
```

### Debug image does not publish

Check:

```bash
ros2 topic info /placement_checker/debug_image -v
ros2 topic hz /placement_checker/debug_image
```

If there is a publisher but no rate, the checker may not be receiving one of:

```text
color image
depth image
camera_info
```

Check:

```bash
ros2 topic info /j100_0897/sensors/camera_0/color/image -v
ros2 topic info /j100_0897/sensors/camera_0/depth/image -v
ros2 topic info /j100_0897/sensors/camera_0/depth/camera_info -v
```

### OpenCV segmentation fault

The checker uses default OpenCV ArUco/AprilTag detector parameters on the robot to avoid a known robot-side OpenCV segmentation fault.

Expected startup warning:

```text
Using default OpenCV ArUco/AprilTag detector parameters to avoid robot-side OpenCV segfault.
```

This warning is expected.

---

## Current Verified Status

The following has been verified on the robot:

```text
AprilTag 16h5 ID 13 detection works.
Camera-frame pose estimation works.
Transform into world frame works when world -> odom is broadcast.
Misplaced service response works.
Properly placed service response works.
```

Correction action integration is the next stage after the reporting pipeline is confirmed.
