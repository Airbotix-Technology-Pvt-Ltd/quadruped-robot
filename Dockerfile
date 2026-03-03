# Use ROS Noetic on Ubuntu 20.04
FROM osrf/ros:noetic-desktop-full

# Avoid interactive prompts during apt-get
ENV DEBIAN_FRONTEND=noninteractive

# Update and install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    build-essential \
    cmake \
    libyaml-cpp-dev \
    libeigen3-dev \
    liblcm-dev \
    libglm-dev \
    libdw-dev \
    libblas-dev \
    liblapack-dev \
    libevdev-dev \
    # ROS 1 dependencies
    ros-noetic-joy \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-rviz \
    # Python dependencies
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-pyqt5 \
    # GUI Support
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglx-mesa0 \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Intel MKL (needed for the BLAS acceleration used in the code)
RUN wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | apt-key add - && \
    echo "deb https://apt.repos.intel.com/oneapi all main" > /etc/apt/sources.list.d/oneAPI.list && \
    apt-get update && apt-get install -y \
    intel-oneapi-mkl-devel \
    && rm -rf /var/lib/apt/lists/*

# Install essential Python packages
RUN pip3 install --no-cache-dir \
    "numpy<2.0" \
    scipy \
    pyyaml \
    psutil

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi && \
    rosdep update

# Pre-setup ROS environment and MKL vars in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/intel/oneapi/setvars.sh > /dev/null 2>&1" >> /root/.bashrc

# Workspace setup
WORKDIR /workspace/quadruped_ws
ENV WORKSPACE_DIR=/workspace/quadruped_ws

# Default command
CMD ["/bin/bash"]
