from kinematics import FiboX_Borot
import pybullet as p
import time

# โหลดหุ่นยนต์
robot = FiboX_Borot()

# ตั้งค่าตำแหน่งและมุมที่ต้องการให้ไป
position = [0.1, 0.1, 0.25]  # ตำแหน่งของ end-effector
orientation = [0, 0, 0]  # มุมของ end-effector (Roll, Pitch, Yaw)

# คำนวณ Inverse Kinematics (IK) เพื่อหามุมข้อต่อ
joint_angles = robot.compute_ink(position, orientation, apply_adjustment=False)
print(joint_angles)

# เช็คว่ามีค่ามุมข้อต่อหรือไม่
if joint_angles:
    joint = [robot.rad_to_deg(i) for i in joint_angles]
    
    print("Moving robot to joint angles:", joint)

    # ใช้ setJointMotorControl2 เพื่อทำให้หุ่นยนต์ขยับ
    for i in range(len(joint_angles)):
        p.setJointMotorControl2(
            bodyIndex=robot.robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_angles[i]
        )

    # ให้เวลาหุ่นยนต์เคลื่อนที่อย่างราบรื่น
    for _ in range(200):  # เพิ่มจำนวนรอบเพื่อให้หุ่นยนต์ขยับต่อเนื่อง
        p.stepSimulation()
        time.sleep(0.005)  # ลดเวลาการหน่วงเพื่อให้การเคลื่อนไหวสมูทขึ้น

# ปิดการเชื่อมต่อ PyBullet (ถ้าไม่ต้องการให้ปิดสามารถคอมเมนต์บรรทัดนี้)
# p.disconnect()
