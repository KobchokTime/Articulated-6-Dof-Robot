FROM tiryoh/ros2-desktop-vnc

# Update package lists and upgrade system
RUN apt-get update && apt-get upgrade -y

# Install additional dependencies for robotic arm simulation
RUN apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    libboost-system-dev \
    build-essential \
    libudev-dev \
    ros-humble-ament-cmake \
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
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create ubuntu user and workspace directory
RUN useradd -m -s /bin/bash ubuntu
RUN mkdir -p /home/ubuntu/robot_ws/ && chown -R ubuntu:ubuntu /home/ubuntu/robot_ws/

# Set working directory for ROS workspace
WORKDIR /home/ubuntu/robot_ws/

# Clone and build dynamixel_hardware package
RUN git clone https://github.com/dynamixel-community/dynamixel_hardware.git -b humble src
RUN vcs import src < src/dynamixel_control.repos

# Debugging: Check the contents of the workspace
RUN ls -la /home/ubuntu/robot_ws/

RUN apt-get update && apt-get upgrade -y
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# ✅ Ensure ROS environment is sourced before build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

# Copy custom .bashrc and additional files (if any)
COPY ros2_ws/.bashrc /home/ubuntu/.bashrc
COPY ros2_ws/.text_art.sh /home/ubuntu/.text_art.sh

# Set up entrypoint
CMD ["/ros_entrypoint.sh"]
