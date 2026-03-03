# Use ROS2 Humble on Ubuntu 22.04 as the starting point
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ── 1. Install System Dependencies ───────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    wget curl git build-essential cmake \
    libyaml-cpp-dev libdw-dev \
    libblas-dev liblapack-dev \
    libglm-dev liblcm-dev \
    python3-pip python3-colcon-common-extensions \
    libevdev-dev \
    libgl1-mesa-glx libgl1-mesa-dri libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo Python bridge and extra tools
RUN pip3 install "numpy<2.0" mujoco scipy

# ── 2. Clone DeepRobotics sdk_deploy for drdds bridge ────────────────────────
WORKDIR /workspace/deps
RUN git clone https://github.com/DeepRoboticsLab/sdk_deploy.git && \
    cd sdk_deploy && \
    git submodule update --init --recursive

# ── 3. Prepare MPC/WBC Core (Tophill Robotics Logic) ─────────────────────────
# We will mount or copy the quadruped-robot source folder. 
# For the Docker build process, we'll assume it's copied into /workspace/quadruped-robot
WORKDIR /workspace/quadruped-robot

# ── 4. Set environment and workspace ─────────────────────────────────────────
WORKDIR /workspace/lite3_ws
COPY src /workspace/lite3_ws/src/quadruped_ros2

# Pre-build setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /root/.bashrc

# ── 5. Setup Build Script ───────────────────────────────────────────────────
# This will be used by the user to build everything inside the container
WORKDIR /workspace
COPY docker/build_all.sh /workspace/build_all.sh
RUN chmod +x /workspace/build_all.sh

WORKDIR /workspace/lite3_ws
CMD ["bash"]
