import pybullet as p
import pybullet_data
import sys
import time
import math
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QFormLayout
from kinematic import FiboX_Borot


class RobotControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.robot = FiboX_Borot(r"C:\\Users\\Toey\\Desktop\\Assem_For_URDF_4\\urdf\\Assem_For_URDF_4.urdf")
        self.initUI()

    def initUI(self):
        self.setWindowTitle("PyBullet Robot Control")
        self.setGeometry(100, 100, 400, 400)
        layout = QVBoxLayout()
        self.inputs = {}
        form_layout = QFormLayout()
        labels = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        for label in labels:
            self.inputs[label] = QLineEdit(self)
            self.inputs[label].setText("0.3" if label in ["X", "Y", "Z"] else "0")
            form_layout.addRow(f"{label}:", self.inputs[label])
        layout.addLayout(form_layout)
        self.move_btn = QPushButton("Move Robot", self)
        self.move_btn.clicked.connect(self.move_robot)
        layout.addWidget(PyQt6self.move_btn)
        self.result_label = QLabel("📍 Actual Position: N/A\n🔩 Joint Angles: N/A", self)
        layout.addWidget(self.result_label)
        self.setLayout(layout)

    def move_robot(self):
        target_position = [float(self.inputs[label].text()) for label in ["X", "Y", "Z"]]
        target_orientation = [self.robot.deg_to_rad(float(self.inputs[label].text())) for label in ["Roll", "Pitch", "Yaw"]]
        # print(target_position)
        # print(target_orientation)
        joint_angles = self.robot.compute_ink(target_position, target_orientation, apply_adjustment=False)
        # print(joint_angles)
        if joint_angles is None:
            self.result_label.setText("❌ IK Failed: No valid solution found!")
            return
        for i in range(len(joint_angles)):
            p.setJointMotorControl2(self.robot.robot_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i])
        for _ in range(200):
            p.stepSimulation()
            time.sleep(0.01)
        actual_position = self.robot.compute_fk(joint_angles)
        joint_angles = self.robot.compute_ink(target_position, target_orientation, apply_adjustment=True)
        joint_angle_str = "\n".join([f"Joint {i+1}: {self.robot.rad_to_deg(joint_angles[i]):.2f}°" for i in range(len(joint_angles))])
        self.result_label.setText(
            f"📍 Actual Position:\nX: {actual_position[0]:.3f}, Y: {actual_position[1]:.3f}, Z: {actual_position[2]:.3f}\n"
            f"Orientation (Euler):\nRoll: {self.robot.rad_to_deg(actual_position[3]):.2f}°, "
            f"Pitch: {self.robot.rad_to_deg(actual_position[4]):.2f}°, "
            f"Yaw: {self.robot.rad_to_deg(actual_position[5]):.2f}°\n\n"
            f"🔩 Joint Angles:\n{joint_angle_str}"
        )
        print("\n✅ หุ่นยนต์เคลื่อนที่ไปยังตำแหน่งที่กำหนดสำเร็จ!")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotControlUI()
    window.show()
    sys.exit(app.exec())