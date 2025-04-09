import pybullet as p
import pybullet_data
import sys
import time
import math
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QFormLayout

# ฟังก์ชันแปลงองศาเป็นเรเดียน
def deg_to_rad(degrees):
    return degrees * (math.pi / 180)

# ฟังก์ชันแปลงเรเดียนเป็นองศา
def rad_to_deg(radians):
    return radians * (180 / math.pi)

class RobotControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.initPyBullet()

    def initUI(self):
        self.setWindowTitle("PyBullet Robot Control")
        self.setGeometry(100, 100, 400, 400)

        layout = QVBoxLayout()

        # อินพุตสำหรับ X, Y, Z, Roll, Pitch, Yaw
        self.inputs = {}
        form_layout = QFormLayout()
        labels = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        for label in labels:
            self.inputs[label] = QLineEdit(self)
            self.inputs[label].setText("0.3" if label in ["X", "Y", "Z"] else "0")  # ค่าเริ่มต้น
            form_layout.addRow(f"{label}:", self.inputs[label])

        layout.addLayout(form_layout)

        # ปุ่ม Move Robot
        self.move_btn = QPushButton("Move Robot", self)
        self.move_btn.clicked.connect(self.move_robot)
        layout.addWidget(self.move_btn)

        # Label แสดงค่าตำแหน่งที่ไปถึงจริง + Joint Angles
        self.result_label = QLabel("📍 Actual Position: N/A\n🔩 Joint Angles: N/A", self)
        layout.addWidget(self.result_label)

        self.setLayout(layout)

    def initPyBullet(self):
        # เริ่ม PyBullet GUI
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # โหลดหุ่นยนต์
        self.robot_id = p.loadURDF(r"C:\Users\Toey\Desktop\Assem_For_URDF_4\urdf\Assem_For_URDF_4.urdf", useFixedBase=True)

        # ปิด Real-time simulation
        p.setRealTimeSimulation(0)

        # ดึงจำนวนข้อต่อ
        self.num_joints = p.getNumJoints(self.robot_id)
        self.end_effector_index = self.num_joints - 1  # กำหนด index ของ End-Effector

    def move_robot(self):
        # ดึงค่าจาก UI
        target_position = [
            float(self.inputs["X"].text()),
            float(self.inputs["Y"].text()),
            float(self.inputs["Z"].text())
        ]
        target_orientation_euler = [
            deg_to_rad(float(self.inputs["Roll"].text())),
            deg_to_rad(float(self.inputs["Pitch"].text())),
            deg_to_rad(float(self.inputs["Yaw"].text()))
        ]
        target_orientation = p.getQuaternionFromEuler(target_orientation_euler)

        # เรียกฟังก์ชันคำนวณ Joint Angles
        joint_angles = self.calculate_joint_angles(target_position, target_orientation)

        if joint_angles is None:
            self.result_label.setText("❌ IK Failed: No valid solution found!")
            return

        # ตั้งค่าข้อต่อให้เคลื่อนที่ไปยังค่าที่คำนวณได้
        for i in range(len(joint_angles)):
            p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i])

        # จำลองการเคลื่อนที่
        for _ in range(200):
            p.stepSimulation()
            time.sleep(0.01)

        # ดึงค่าตำแหน่งจริงหลังจากเคลื่อนที่
        actual_position, actual_orientation = p.getLinkState(self.robot_id, self.end_effector_index)[:2]
        actual_orientation_euler = p.getEulerFromQuaternion(actual_orientation)

        # ดึงค่ามุมข้อต่อจริงหลังจากเคลื่อนที่
        actual_joint_angles = [p.getJointState(self.robot_id, i)[0] for i in range(len(joint_angles))]
        actual_joint_angles = list(actual_joint_angles)
        
        actual_joint_angles[2] = -actual_joint_angles[2]
        actual_joint_angles[3] = -actual_joint_angles[3]
        actual_joint_angles[4] = actual_joint_angles[4] - actual_joint_angles[3]

        # อัปเดต UI ให้แสดงค่าจริงที่ได้
        joint_angle_str = "\n".join([f"Joint {i+1}: {rad_to_deg(actual_joint_angles[i]):.2f}°" for i in range(len(actual_joint_angles))])

        self.result_label.setText(
            f"📍 Actual Position:\n"
            f"X: {actual_position[0]:.3f}, Y: {actual_position[1]:.3f}, Z: {actual_position[2]:.3f}\n"
            f"Orientation (Euler):\n"
            f"Roll: {rad_to_deg(actual_orientation_euler[0]):.2f}°, "
            f"Pitch: {rad_to_deg(actual_orientation_euler[1]):.2f}°, "
            f"Yaw: {rad_to_deg(actual_orientation_euler[2]):.2f}°\n\n"
            f"🔩 Joint Angles:\n{joint_angle_str}"
        )

        print("\n✅ หุ่นยนต์เคลื่อนที่ไปยังตำแหน่งที่กำหนดสำเร็จ!")

    def calculate_joint_angles(self, position, orientation):
        """
        คำนวณ Inverse Kinematics (IK)
        รับ input: Position (X, Y, Z) และ Orientation (Quaternion)
        Return: Joint Angles หรือ None ถ้าไม่มีคำตอบ
        """
        joint_angles = p.calculateInverseKinematics(self.robot_id, self.end_effector_index, position, orientation)

        if joint_angles is None or len(joint_angles) == 0:
            return None  # กรณี IK หาคำตอบไม่ได้

        return joint_angles

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotControlUI()
    window.show()
    sys.exit(app.exec())
