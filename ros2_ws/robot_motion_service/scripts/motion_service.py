#!/usr/bin/env python3

import rclpy
import math
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from robot_motion_service.srv import SetPosition
import subprocess


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for velocity controller
        self.vel_publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Action client for trajectory controller
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        # Custom service to set target positions
        self.position_service = self.create_service(SetPosition, 'set_position', self.set_position_service)

        # Gear reduction ratio
        self.gear_ratio = 6.3

        # Store current joint positions
        self.current_joint_positions = {}

        # Initialize robot position at startup
        self.initialize_robot_position()

        self.get_logger().info("Robot Controller initialized and ready to work!")

    def joint_state_callback(self, msg):
        """Callback function to update current joint positions."""
        self.current_joint_positions = {name: pos for name, pos in zip(msg.name, msg.position)}

    def initialize_robot_position(self):
        """Set all joint positions to 0 and switch to velocity controller."""
        try:
            # Step 1: Switch to joint_trajectory_controller
            self.switch_controller('joint_trajectory_controller', activate=True)

            # Step 2: Send initial trajectory using Action Interface
            self.send_initial_trajectory()

            # Step 3: Wait for robot to reach position
            self.get_logger().info("Waiting for robot to reach initial position...")
            time.sleep(3)  # Wait 3 seconds

            # Step 4: Switch to velocity_controller
            self.switch_controller('velocity_controller', activate=True)

        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")

    def switch_controller(self, controller_name, activate):
        """Switch controllers between trajectory and velocity modes."""
        if activate:
            if controller_name == 'joint_trajectory_controller':
                cmd = [
                    'ros2', 'control', 'switch_controllers',
                    '--activate', 'joint_state_broadcaster',
                    '--activate', 'joint_trajectory_controller',
                    '--deactivate', 'velocity_controller'
                ]
            elif controller_name == 'velocity_controller':
                cmd = [
                    'ros2', 'control', 'switch_controllers',
                    '--activate', 'joint_state_broadcaster',
                    '--activate', 'velocity_controller',
                    '--deactivate', 'joint_trajectory_controller'
                ]
            else:
                raise ValueError(f"Unknown controller: {controller_name}")
        else:
            raise ValueError("Deactivate option is not supported in this example.")

        subprocess.run(cmd, check=True)
        self.get_logger().info(f"Switched to {controller_name}")

    def send_initial_trajectory(self):
        """Send initial trajectory to move all joints to position 0."""
        # Wait for action server to be ready
        self.trajectory_client.wait_for_server()

        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']

        # Set the target positions and duration
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # All joints to 0
        point.time_from_start.sec = 3  # 3 seconds to reach
        goal_msg.trajectory.points = [point]

        # Send goal to action server
        self.get_logger().info("Sending trajectory to move joints to initial position...")
        self.trajectory_client.send_goal_async(goal_msg)

    def shortest_path(self, current, target):
        """Find the shortest path between two angles considering -180 to 180 range"""
        delta = target - current
        if delta > 180:
            return target - 360  # Move counterclockwise
        elif delta < -180:
            return target + 360  # Move clockwise
        return target

    def set_position_service(self, request, response):
        """Handle custom service to set target positions."""
        try:
            # Target positions from service (degrees)
            target_degrees = request.target_positions
            target_radians = [math.radians(deg) for deg in target_degrees]  # Convert to radians

            # Ensure joint states are available
            if not self.current_joint_positions:
                self.get_logger().error("No joint state data available yet!")
                response.success = False
                response.message = "No joint state data available"
                return response

            # Movement time (seconds)
            move_time = 5.0

            # Compute required velocities
            velocity_radians_per_sec = []
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']

            for i, joint_name in enumerate(joint_names):
                if joint_name in self.current_joint_positions:
                    # 🔹 แปลงค่าที่อ่านจาก `/joint_states` กลับเป็นค่าที่แท้จริงก่อนคำนวณ
                    current_pos_raw = self.current_joint_positions[joint_name]
                    current_pos = math.degrees(current_pos_raw / self.gear_ratio)  # แปลงค่าจริงจากมอเตอร์เป็นองศา
                
                    # 🔹 คำนวณเป้าหมายโดยใช้ `shortest_path()`
                    adjusted_target = self.shortest_path(current_pos, target_degrees[i])
                    target_radians[i] = math.radians(adjusted_target)  # แปลงกลับเป็น radian
                
                    # คำนวณความเร็วที่ต้องใช้
                    velocity = ((target_radians[i] - math.radians(current_pos)) / move_time) * self.gear_ratio
                    velocity_radians_per_sec.append(velocity)
                else:
                    velocity_radians_per_sec.append(0.0)

            # Publish velocity commands
            self.publish_velocity(velocity_radians_per_sec)

            # Continuously check until the joints reach the target
            start_time = time.time()
            while time.time() - start_time < move_time:
                if all(
                    abs((math.degrees(self.current_joint_positions[joint_name] / self.gear_ratio)) - target_degrees[i]) < 2.0
                    for i, joint_name in enumerate(joint_names) if joint_name in self.current_joint_positions
                ):
                    break
                time.sleep(0.1)  # Check every 100ms

            # Stop the robot
            self.publish_velocity([0.0] * len(joint_names))

            response.success = True
            response.message = "Position command executed successfully."
        except Exception as e:
            self.get_logger().error(f"Failed to execute position command: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    def publish_velocity(self, velocities):
        """Publish velocity commands to the velocity controller."""
        vel_msg = Float64MultiArray()
        vel_msg.data = velocities
        self.vel_publisher.publish(vel_msg)
        self.get_logger().info(f"Published velocity: {velocities}")


def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

