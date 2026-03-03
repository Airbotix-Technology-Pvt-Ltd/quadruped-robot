#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# ── 1. Build drdds (DeepRobotics core messages) ───────────────────────────────────
echo "--- Building drdds messages ---"
cd /workspace/deps/sdk_deploy
colcon build --packages-select drdds lite3_sdk_deploy --cmake-args -DBUILD_PLATFORM=x86
source install/setup.bash

# ── 2. Build our MPC/WBC Gazebo/SDK wrapper ──────────────────────────────────────
echo "--- Building MPC/WBC Robot Controller ---"
cd /workspace/lite3_ws
colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DBUILD_PLATFORM=x86

echo "--- Build finished successfully! ---"
echo "To run simulation:"
echo "Terminal 1: bash /workspace/docker/launch_mujoco.sh"
echo "Terminal 2: source install/setup.bash && ros2 run quadruped_ros2 mpc_runner"
