# Use the base image that already contains ROS 2 Humble and Ubuntu 22.04
FROM tiryoh/ros2-desktop-vnc

# Disable problematic repository temporarily and install basic packages first
RUN mv /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list.bak 2>/dev/null || true && \
    apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    libboost-system-dev \
    build-essential \
    libudev-dev \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Fix ROS 2 repository with proper GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2-new.list > /dev/null && \
    apt-get update

# Now install ROS 2 packages
RUN apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-resources-prbt-moveit-config \
    ros-humble-moveit-visual-tools \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-ros-testing \
    ros-humble-dynamixel-sdk \
    ros-humble-py-binding-tools \
    ros-humble-rviz2 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install pybullet ttkbootstrap paho-mqtt

# Set environment variables for the robot model
ENV ROBOT_MODEL=articulated_arm

# Set working directory
WORKDIR /home/ubuntu

# Copy workspace files
COPY ros2_ws/.bashrc /home/ubuntu/.bashrc
COPY ros2_ws/.text_art.sh /home/ubuntu/.text_art.sh
COPY ros2_ws /home/ubuntu/robot_ws

# Setup ROS environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/robot_ws/install/setup.bash" >> /home/ubuntu/.bashrc

# Setup robot_ws
WORKDIR /home/ubuntu/robot_ws

# Import repositories and install dependencies
RUN vcs import src < src/dynamixel_control.repos && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="joint_state_publisher_gui"

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Make all .py scripts executable (after colcon build is done)
RUN find src/robot_motion_service/scripts/ -name "*.py" -exec chmod +x {} \;

# Source the workspace (optional, for interactive use)
RUN echo "source /home/ubuntu/robot_ws/install/setup.bash" >> /home/ubuntu/.bashrc

# Set entrypoint
CMD ["bash"]