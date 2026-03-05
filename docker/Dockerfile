# Use official ROS 2 Humble base image
FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-dev \
    python3-numpy \
    libtbb-dev \
    libyaml-cpp-dev \
    libglfw3-dev \
    liblcm-dev \
    pkg-config \
    x11-apps \
    mesa-utils \
    libpython3-dev \
    unzip \
    wget \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 2. Install ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-msgs \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# 3. Setup non-root user (Generic name to avoid host-specific paths)
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 4. Download MuJoCo and LibTorch runtimes to a system directory
# This avoids "shadowing" when mounting the local workspace volume
RUN mkdir -p /opt/rl_hnav/library
COPY src/rl_sar/run/download_mujoco.sh src/rl_sar/run/download_inference_runtime.sh src/rl_sar/run/common.sh /tmp/scripts/
RUN cd /tmp/scripts && \
    bash download_mujoco.sh /opt/rl_hnav/library && \
    bash download_inference_runtime.sh /opt/rl_hnav/library/inference_runtime libtorch && \
    chown -R $USERNAME:$USERNAME /opt/rl_hnav && \
    rm -rf /tmp/scripts

# 5. Env variables
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

USER $USERNAME
WORKDIR /workspace

# 6. Automatic sourcing and runtime library linking
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/rl_hnav/library/mujoco/lib" >> ~/.bashrc
# Map system libraries to the expected project structure via symlinks
RUN echo "mkdir -p /workspace/library && ln -s /opt/rl_hnav/library/* /workspace/library/ 2>/dev/null || true" >> ~/.bashrc

CMD ["/bin/bash"]
