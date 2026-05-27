#!/bin/bash
# Runs on the Jetson INSIDE the jazzy distrobox. Launches the realsense +
# block_localization + block_viz stack, plus a dummy world-frame pose for
# testing. Each subprocess is detached via setsid + stdin redirect so it
# survives the parent shell exiting when the SSH session that invoked us
# disconnects.
#
# Usage:  launch_remote.sh [namespace]

set -e

NS="${1:-sierra}"

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/pioneer_scouts/install/setup.bash" ]; then
    source "$HOME/pioneer_scouts/install/setup.bash"
else
    echo "ERROR: $HOME/pioneer_scouts/install/setup.bash not found. Build must succeed first."
    exit 1
fi

# Stop any previous instance.
pkill -f "block_localization_launch.py"   2>/dev/null || true
pkill -f "block_viz_launch.py"            2>/dev/null || true
pkill -f "ros2 topic pub.*${NS}/pose"     2>/dev/null || true
pkill -f "realsense2_camera_node"         2>/dev/null || true
sleep 1

# Realsense + block_localization
setsid bash -c "ros2 launch block_localization block_localization_launch.py namespace:=${NS}" \
    > /tmp/block_localization.log 2>&1 < /dev/null &
disown

# Block visualization
setsid bash -c "ros2 launch block_localization block_viz_launch.py namespace:=${NS}" \
    > /tmp/block_viz.log 2>&1 < /dev/null &
disown

# Optional dummy map-frame pose for bring-up. Gate on DUMMY_POSE=1 so it
# does not race a real BEV publisher once that is online.
if [ "${DUMMY_POSE:-0}" = "1" ]; then
    sleep 3
    setsid bash -c "ros2 topic pub --rate 5 /${NS}/pose geometry_msgs/Pose \
        '{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}'" \
        > /tmp/dummy_pose.log 2>&1 < /dev/null &
    disown
fi

echo "block_localization stack launched, namespace=${NS}"
