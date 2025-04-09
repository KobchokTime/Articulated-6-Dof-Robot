import pybullet as p
import pybullet_data
import time
import math

# ฟังก์ชันแปลงองศาเป็นเรเดียน
def deg_to_rad(degrees):
    return degrees * (math.pi / 180)

# เริ่ม PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 🔹 หมุนหุ่นยนต์ให้ "ตั้งขึ้น" ตามแนวแกน Z
rotation_angle = 0  # มุมเริ่มต้นของการหมุน
base_rotation = p.getQuaternionFromEuler((0, -math.pi / 2, 0))  # ตั้งค่าหมุนเริ่มต้น

# โหลดหุ่นยนต์
robot_id = p.loadURDF(r"C:\Users\Toey\Desktop\Assem_For_URDF_4\urdf\Assem_For_URDF_4.urdf",
                       basePosition=[0, 0, 0],
                       baseOrientation=base_rotation,
                       useFixedBase=True)

# ปิด Real-time simulation
p.setRealTimeSimulation(0)

# 🔍 ซูมเข้าไปใกล้ขึ้น
target_position = [0, 0, 0.4]
camera_distance = 0.3
camera_yaw = 90
camera_pitch = -45
p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, target_position)

# ✅ เปิด GUI ค้างไว้และรองรับการหมุนด้วยปุ่มกด
while True:
    keys = p.getKeyboardEvents()

    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_WAS_TRIGGERED:
        rotation_angle += math.pi / 8  # หมุนซ้าย 22.5°
    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_WAS_TRIGGERED:
        rotation_angle -= math.pi / 8  # หมุนขวา 22.5°

    # อัปเดตการหมุนของหุ่นยนต์
    new_rotation = p.getQuaternionFromEuler((0, -math.pi / 2, rotation_angle))
    p.resetBasePositionAndOrientation(robot_id, [0, 0, 0], new_rotation)

    # รัน Simulation
    p.stepSimulation()
    time.sleep(1 / 240)
