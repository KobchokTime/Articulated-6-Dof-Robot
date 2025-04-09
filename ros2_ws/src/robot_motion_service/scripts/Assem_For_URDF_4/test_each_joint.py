import pybullet as p
import pybullet_data
import time
import math

# ฟังก์ชันแปลงองศาเป็นเรเดียน
def deg_to_rad(degrees):
    return degrees * (math.pi / 180)

# ฟังก์ชันแปลงเรเดียนเป็นองศา
def rad_to_deg(radians):
    return radians * (180 / math.pi)

# เริ่ม PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# โหลดหุ่นยนต์
robot_id = p.loadURDF(r"C:\Users\Toey\Desktop\Assem_For_URDF_4\urdf\Assem_For_URDF_4.urdf", useFixedBase=True)

# ปิด Real-time simulation
p.setRealTimeSimulation(0)

# ตรวจสอบจำนวนข้อต่อ
num_joints = p.getNumJoints(robot_id)
print(f"Total Joints: {num_joints}")

# ดึงข้อมูลขีดจำกัดของข้อต่อ
joint_limits = {
    'joint1': (-134, 134),
    'joint2': (-90, 0),
    'joint3': (-90, 45),
    'joint4': (-180, 180),
    'joint5': (-90, 90), 
    'joint6': (-180, 180)
}

# ตั้งค่าท่าเริ่มต้น (ให้ทุก Joint เริ่มที่ 0°)
for i in range(num_joints):
    p.resetJointState(robot_id, i, 0)

# **ทดสอบการหมุนแต่ละข้อต่อ**
for joint_index in range(num_joints):
    joint_name = f"Joint {joint_index + 1}"
    lower_limit, upper_limit = joint_limits[f'joint{joint_index + 1}']
    
    print(f"\n🔍 **Testing {joint_name}**")
    print(f"   Limits: {lower_limit}° to {upper_limit}°")

    # **ค่อยๆ หมุนจาก Lower ไป Upper แล้วกลับมา**
    for angle in range(lower_limit, upper_limit + 1, 10):  # เพิ่มทีละ 10°
        rad_angle = deg_to_rad(angle)
        p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=rad_angle)
        p.stepSimulation()
        time.sleep(0.05)

    # **หมุนกลับไปที่ 0°**
    p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=0)
    for _ in range(30):
        p.stepSimulation()
        time.sleep(0.05)

    print(f"✅ **{joint_name} Test Complete!**")

p.disconnect()
