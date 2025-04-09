#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from robot_motion_service.srv import SolveIK
from kinematic_fibox import FiboX_Borot

class KinematicsService(Node):
    def __init__(self):
        super().__init__('kinematics_service')
        
        # Create the kinematics solver
        self.kinematics = FiboX_Borot()
        
        # Create the service
        self.service = self.create_service(
            SolveIK, 
            'solve_ik', 
            self.solve_ik_callback
        )
        
        self.get_logger().info('Kinematics Service is ready')
    
    def solve_ik_callback(self, request, response):
        """Handle inverse kinematics service requests."""
        try:
            # Extract the target position and orientation from the request
            x = request.x
            y = request.y
            z = request.z
            roll = request.rx
            pitch = request.ry
            yaw = request.rz
            
            position = [x/1000, y/1000, z/1000]
            orientation = [roll, pitch, yaw]
            
            # Log input values
            self.get_logger().info(f"🔹 Received IK request:")
            self.get_logger().info(f"   - Position (mm): x={x}, y={y}, z={z}")
            self.get_logger().info(f"   - Orientation (rad): roll={roll}, pitch={pitch}, yaw={yaw}")

            # Compute inverse kinematics
            ik_solutions = self.kinematics.compute_ink(position, orientation, apply_adjustment=False)
            
            # Log IK solver inputs and outputs
            self.get_logger().info(f"🔸 Sent to IK Solver:")
            self.get_logger().info(f"   - Position (m): x={x:.3f}, y={y:.3f}, z={z:.3f}")
            self.get_logger().info(f"   - Orientation (rad): roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")

            if not ik_solutions:
                response.success = False
                response.message = "No IK solution found for the given pose"
                response.joint_angles = []
                self.get_logger().warning("❌ No IK solution found.")
                return response
            
            # Get the first solution (or implement a selection strategy)
            best_solution = ik_solutions  

            # ✅ Send joint angles in radians (NO conversion to degrees)
            response.success = True
            response.message = "IK solution found successfully"
            response.joint_angles = best_solution  # Send radians directly
            
            # Log IK output (joint angles in radians)
            self.get_logger().info(f"✅ Found IK Solution (radians): {best_solution}")

        except Exception as e:
            self.get_logger().error(f"❌ Error solving IK: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
            response.joint_angles = []
        
        return response

def main():
    rclpy.init()
    node = KinematicsService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
