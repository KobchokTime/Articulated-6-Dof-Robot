import pybullet as p
import pybullet_data
import math
from ament_index_python.packages import get_package_share_directory
import os

class FiboX_Borot:
    def __init__(self):
        urdf_path = os.path.join(get_package_share_directory('robot_motion_service'), 'urdf', 'Assem_For_URDF_4.urdf')
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)
        p.setRealTimeSimulation(0)  
        self.num_joints = p.getNumJoints(self.robot_id)
        self.end_effector_index = self.num_joints - 1
        
        # Joint limits (adjust as needed)
        self.joint_limits_low = [-3.14 , -1.57, -1.57, -3.14, -3.14, -3.14]
        self.joint_limits_high = [3.14 , 1.57, 1.57, 3.14, 3.14, 3.14]
        self.joint_damping = [0.01] * self.num_joints
        
        # # Debug: Check joint limits from URDF
        # for i in range(self.num_joints):
        #     joint_info = p.getJointInfo(self.robot_id, i)
        #     print(f"Joint {i}: Lower Limit = {joint_info[8]}, Upper Limit = {joint_info[9]}")

    def deg_to_rad(self, degrees):
        return degrees * (math.pi / 180)

    def rad_to_deg(self, radians):
        return radians * (180 / math.pi)

    def compute_fk(self, joint_angles):
        for i in range(len(joint_angles)):
            p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i])
        p.stepSimulation()
        actual_position, actual_orientation = p.getLinkState(self.robot_id, self.end_effector_index)[:2]
        actual_orientation_euler = p.getEulerFromQuaternion(actual_orientation)
        return (
            actual_position[0], actual_position[1], actual_position[2],
            actual_orientation_euler[0], actual_orientation_euler[1], actual_orientation_euler[2]
        )

    def compute_ink(self, position, orientation, apply_adjustment=False):
        target_orientation = p.getQuaternionFromEuler(orientation)
        joint_angles = p.calculateInverseKinematics(
            self.robot_id, self.end_effector_index, position, target_orientation,
            lowerLimits=self.joint_limits_low,
            upperLimits=self.joint_limits_high,
            jointRanges=[high - low for low, high in zip(self.joint_limits_low, self.joint_limits_high)],
            restPoses=[0] * self.num_joints,
            jointDamping=self.joint_damping,
            maxNumIterations=100
        )
        
        if joint_angles and apply_adjustment:
            joint_angles = self.adjust_joint_angles_for_sim(joint_angles)
        return joint_angles
    
    def adjust_joint_angles_for_sim(self, joint_angles):
        adjusted_angles = list(joint_angles)
        adjusted_angles[2] = -adjusted_angles[2]  # Adjust Joint 3
        adjusted_angles[3] = -adjusted_angles[3]  # Adjust Joint 4
        adjusted_angles[4] = adjusted_angles[4] - adjusted_angles[3]  # Adjust Joint 5
        return adjusted_angles
