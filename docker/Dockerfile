# Use official ROS 2 Humble base image
FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install system dependencies required for compilation and scripts
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

# 2. Install ROS 2 dependencies
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
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    ros-humble-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# 3. Setup non-root user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 4. Hardware acceleration and Display envs
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

USER $USERNAME
WORKDIR /workspace

# 5. Automatic sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
