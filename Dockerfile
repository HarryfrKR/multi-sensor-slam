# Use the official ROS 2 Humble image as the base
FROM osrf/ros:humble-desktop-full

ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# install basic libraries
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    pkg-config \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    python3-vcstool \
    python3-colcon-common-extensions


# Install ORB-SLAM3 dependencies
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev \
    libglew-dev \
    libqt5opengl5-dev \
    libboost-all-dev \
    freeglut3-dev \
    libsuitesparse-dev \
    libatlas-base-dev \
    liblapacke-dev \
    libeigen3-dev \
    libxkbcommon-x11-0 \
    libyaml-cpp-dev \
    libceres-dev \
    ffmpeg \
    gdb

# Install ROS2 Packages
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    ros-humble-ros-gz \
    ros-humble-slam-toolbox \
    ros-humble-rplidar-ros \
    ros-humble-cartographer-ros \
    ros-humble-ign-ros2-control \
    ros-humble-irobot-create-msgs \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-irobot-create-gazebo-sim \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-image-transport \
    ros-humble-nav2-bringup \
    ros-humble-libg2o \
    ros-humble-sophus \
    ros-humble-image-transport \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-ros2bag \
    ros-humble-tf-transformations

# Install Ignition Fortress Gazebo
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y \
    ignition-fortress

# User variables
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

ENV ROS_DOMAIN_ID=42
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS environment by default
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/vscode/.bashrc