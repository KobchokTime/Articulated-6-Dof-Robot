FROM tiryoh/ros2-desktop-vnc

# Install additional dependencies for robotic arm simulation
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    libboost-system-dev \
    build-essential \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages required for robotic arm simulation
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-resources-prbt-moveit-config \
    ros-humble-moveit-visual-tools \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-dynamixel-sdk \
    && rm -rf /var/lib/apt/lists/*

# Install additional visualization tools
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables for the robot model
ENV ROBOT_MODEL=articulated_arm

# Preload .bashrc with ROS environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/ubuntu/robot_ws/install/setup.bash" >> ~/.bashrc

# Clone and build dynamixel_hardware package
RUN mkdir -p ~/ros/humble && cd ~/ros/humble && \
    git clone https://github.com/dynamixel-community/dynamixel_hardware.git -b humble src && \
    vcs import src < src/dynamixel_control.repos && \
    apt-get update && apt-get upgrade -y && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
    . install/setup.bash

# Copy custom .bashrc and additional files (if any)
COPY ros2_ws/.bashrc /home/ubuntu/.bashrc
COPY ros2_ws/.text_art.sh /home/ubuntu/.text_art.sh

# Set up entrypoint
CMD ["/ros_entrypoint.sh"]
