# Use the base image that already contains ROS 2 Humble and Ubuntu 22.04
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
    ros-humble-ros-testing\
    ros-humble-dynamixel-sdk \
    && rm -rf /var/lib/apt/lists/*
    
# Install py_binding_tools
RUN apt-get update && apt-get install -y ros-humble-py-binding-tools && \
    rm -rf /var/lib/apt/lists/*

# Install additional visualization tools
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables for the robot model
ENV ROBOT_MODEL=articulated_arm

COPY ros2_ws/.bashrc /home/ubuntu/.bashrc
COPY ros2_ws/.text_art.sh /home/ubuntu/.text_art.sh
COPY ros2_ws /home/ubuntu/robot_ws
# Preload .bashrc with ROS environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/ubuntu/robot_ws/install/setup.bash" >> ~/.bashrc
    

    
# Navigate to moveit2_tutorials and install dependencies
WORKDIR /home/ubuntu/robot_ws/src/moveit2_tutorials
RUN apt-get update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# Build the ROS 2 workspace
WORKDIR /home/ubuntu/robot_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --base-path /home/ubuntu/robot_ws --parallel-workers 1


# Copy robot description and simulation files (if available)
#COPY ./robot_description /home/ubuntu/robot_description
#COPY ./simulation /home/ubuntu/simulation

# Set up entrypoint
CMD ["/ros_entrypoint.sh"]
