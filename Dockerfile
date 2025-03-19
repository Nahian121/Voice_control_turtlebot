# Use the official ROS 2 Humble image
FROM osrf/ros:humble-desktop

# Set environment variable
ENV TURTLEBOT3_MODEL=burger
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt update && apt install -y \
    wget \
    python3-pip \
    portaudio19-dev \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-turtlebot3-bringup \
    ros-humble-gazebo-ros \
    libx11-dev \
    x11-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    speechrecognition \
    pyaudio

# Set up the workspace
WORKDIR /root/tb_ws
COPY src/vo_co_bot /root/tb_ws/src/vo_co_bot

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Source workspace on start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/tb_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]
