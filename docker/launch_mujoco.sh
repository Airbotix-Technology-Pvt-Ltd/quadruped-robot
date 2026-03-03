#!/bin/bash
# launch_mujoco.sh — starts MuJoCo sim within Lite3_MPC_WBC container

source /opt/ros/humble/setup.bash
source /workspace/deps/sdk_deploy/install/setup.bash

export ROS_DOMAIN_ID=1

SIM_SCRIPT="/workspace/deps/sdk_deploy/src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py"

echo "Starting MuJoCo simulation for Lite3 MPC/WBC..."
echo "(Run MPC in shell 2: docker exec -it lite3_mpc_worker bash -c 'source install/setup.bash && ros2 run quadruped_ros2 mpc_runner')"
echo ""

python3 "$SIM_SCRIPT"
