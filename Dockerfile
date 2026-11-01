# Target amd64 as official ROS Melodic images lack ARM support (suppresses warning)
ARG PLATFORM=linux/amd64

# Base image includes ROS Melodic, Gazebo 9.0.0, and RViz (Ubuntu 18.04)
FROM --platform=$PLATFORM osrf/ros:melodic-desktop-full

# Install catkin and build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python-catkin-tools \
    python-rosdep \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set working directory for repository mount
WORKDIR /workspace

# Source global ROS setup script automatically
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Optional: Source local catkin workspace
# RUN echo "source /workspace/devel/setup.bash" >> ~/.bashrc

# Default to interactive shell
CMD ["bash"]