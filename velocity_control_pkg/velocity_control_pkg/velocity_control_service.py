#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from example_interfaces.srv import Float32MultiArray  # Service interface
import numpy as np

class VelocityControlService(Node):
    def __init__(self):
        super().__init__('velocity_control_service')
        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.current_positions = [0.0] * 6  # ตำแหน่งปัจจุบันของ joint ทั้งหมด
        self.total_time = 5.0  # เวลาที่ต้องการให้ทุกแกนจบพร้อมกัน (วินาที)
        self.gear_ratio = 6.3  # อัตราทด 6.3:1

        # สร้าง service สำหรับรับค่ามุมเป้าหมาย
        self.srv = self.create_service(
            Float32MultiArray, 'set_joint_angles', self.set_joint_angles_callback)

    def joint_state_callback(self, msg):
        # อ่านค่าตำแหน่งปัจจุบันจาก /joint_states
        self.current_positions = list(msg.position)

    def set_joint_angles_callback(self, request, response):
        # รับค่ามุมเป้าหมายจากผู้ใช้
        target_angles_deg = request.data  # ค่าที่ผู้ใช้ป้อนมาในองศา
        if len(target_angles_deg) != 6:
            self.get_logger().error("Please provide exactly 6 joint angles.")
            response.success = False
            return response

        # แปลงเป้าหมายจากองศาเป็น Radians
        target_angles_rad = [np.radians(angle) for angle in target_angles_deg]

        # คำนวณระยะทาง (error) และปรับด้วยอัตราทด
        errors = [(target - current) * self.gear_ratio for target, current in zip(target_angles_rad, self.current_positions)]

        # คำนวณความเร็วที่ต้องการในแต่ละแกน
        velocities = [error / self.total_time for error in errors]  # ความเร็ว = ระยะทาง / เวลา

        # ส่งคำสั่ง velocity ไปยัง Velocity Controller
        msg = Float64MultiArray()
        msg.data = velocities
        self.publisher.publish(msg)
        self.get_logger().info(f"Sending velocities (with gear ratio): {velocities}")

        # รอให้หุ่นเคลื่อนที่จนถึงเป้าหมาย
        self.get_logger().info(f"Waiting for {self.total_time} seconds for the joints to reach the target...")
        self.get_clock().sleep_for(self.total_time)

        # หยุดการเคลื่อนที่เมื่อถึงเป้าหมาย
        self.stop_motion()

        response.success = True
        return response

    def stop_motion(self):
        # ส่งคำสั่ง velocity = 0 เพื่อหยุดการเคลื่อนที่
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self.publisher.publish(msg)
        self.get_logger().info("Stopping motion (all velocities = 0).")

def main(args=None):
    rclpy.init(args=args)
    velocity_control_service = VelocityControlService()

    try:
        rclpy.spin(velocity_control_service)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        velocity_control_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

