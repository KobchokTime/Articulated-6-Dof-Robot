# Use the base image that already contains ROS 2 Humble and Ubuntu 22.04
FROM tiryoh/ros2-desktop-vnc

# Install additional dependencies for robotic arm simulation
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    libboost-system-dev \
    build-essential \
    libudev-dev \
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
