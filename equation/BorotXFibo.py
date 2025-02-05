import numpy as np
from scipy.spatial.transform import Rotation as R

class FiboX_Borot:
    def __init__(self):
        ##### set parameter of robot 6 DOF (mm)
        self.d3 = 14.86
        self.l3 = 160
        self.l4 = 35.42
        self.l5 = 113.56
        self.l45 = self.l4 + self.l5
        self.l2 = 37.5
        self.d2 = 50.6
        self.d1 = 39.8
        self.l1 = 167.6
        self.le = 10
        self.l6 = 19
    # === Define DH Parameters ===
    def dh_transform(self, a, alpha, d, theta):
        """Compute DH Transformation Matrix."""
        Tx = np.array([[1, 0, 0, a],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        Rx = np.array([[1, 0, 0, 0],
                       [0, np.cos(alpha), -np.sin(alpha), 0],
                       [0, np.sin(alpha), np.cos(alpha), 0],
                       [0, 0, 0, 1]])
        Tz = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, d],
                       [0, 0, 0, 1]])
        Rz = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                       [np.sin(theta), np.cos(theta), 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        
        return Tx @ Rx @ Tz @ Rz
    def compute_fk(self, joint_angles):
        DH_params = [
                    [0, 0, self.l1, 0],
                    [self.l2, np.pi/2, self.d1, np.pi/2],
                    [self.l3, 0, -self.d2, 0],
                    [self.d3, np.pi/2, self.l45, 0],
                    [0, -np.pi/2, 0, 0],
                    [0, np.pi/2, self.l6, 0]
                    ]
        """Compute joint positions using forward kinematics."""
        T = np.eye(4)
        joints = []
        # print(T)
        
        for i in range(6):
            a, alpha, d, theta = DH_params[i]
            theta1 = joint_angles[i]
            T = T @ self.dh_transform(a, alpha, d, theta + theta1)
            # print(T)
            # print("------------------")
            joints.append(np.array(T))
        T6e = np.array([[0, 0, 1, 0],
                        [0, -1, 0, 0],
                        [1, 0, 0, self.le],
                        [0, 0, 0, 1]])
        T = T @ T6e
        joints.append(np.array(T))
        
        return np.array(joints)
    
    def compute_ink(self, xe, ye, ze, roll, pitch, yaw, B=[1,0,1]):
        rpy = np.radians([roll, pitch, yaw])
        R0e = R.from_euler('xyz', rpy).as_matrix()
        p0e = np.array([[xe], [ye], [ze]]) 
        de = np.array([[self.le], [0], [0]])
        
        wrist = p0e - R0e @ de
        xw,yw,zw = [wrist[0,0], wrist[1,0], wrist[2,0]]
        #### Inverse position kinematic 0 - wrist => q1, q2, q3
        B1 = [-1,1]
        B2 = [-1,1]
        B3 = [-1,1]
        k2 = self.d2 - self.d1

        result = []

        if ((xw**2)+(yw**2)-(k2**2)) >= 0:
            for b1 in B1:
                m = b1*np.sqrt((xw**2)+(yw**2)-(k2**2)) - self.l2
                n = zw - self.l1
                e = (self.d3**2) + (self.l45**2) + (self.l3**2)
                f = (m**2) + (n**2) - e

                a = (self.d3**2)*(self.l3**2) + (self.l45**2)*(self.l2**2)
                b = -2*f*self.d3*self.l3
                c = (f**2) - (self.l45**2)*(self.l2**2)
                c3 = np.roots([a,b,c])
                if np.all(np.isreal(c3)):
                    for r in c3:
                        for b2 in B2:
                            q3 = np.arctan2(b2*np.sqrt(1-r),r) # theta3 result (rad)
                            
                            a1 = self.d3*np.cos(q3) + self.l45*np.sin(q3) + self.l3
                            a2 = self.l45*np.cos(q3) - self.d3*np.sin(q3)
                            detA = -(a1**2)-(a2**2)
                            if detA != 0:
                                s2 = (a1*m - a2*n)/detA
                                c2 = (-a2*m - a1*n)/detA
                                q2 = np.arctan2(s2,c2) # theta2 result (rad)
                                
                                k1 = -self.d3*np.sin(q2+q3) + self.l45*np.cos(q2+q3) - self.l3*np.sin(q2) + self.l2
                                gamma = np.arctan2(k2,k1)
                                
                                q1 = np.arctan2(yw,xw) - gamma # theta1 result (rad)
                                
                                
                                
                                #### Forward orientation matrix => R03
                                R03 = np.array([[-np.cos(q1)*np.sin(q2+q3), -np.cos(q1)*np.cos(q2+q3), np.sin(q1)],
                                                [-np.sin(q1)*np.sin(q2+q3), -np.sin(q1)*np.cos(q2+q3), -np.cos(q1)],
                                                [np.cos(q2+q3), -np.sin(q2+q3), 0]])
                                R03_T = R03.T
                                # print(R03)
                                #### Result of orientation e เทียบ 3
                                R3e = R03_T @ R0e
                                
                                c5 = -R3e[1,0]
                                for b3 in B3:
                                    s5 = b3*np.sqrt((R3e[1,1]**2) + (R3e[1,2]**2))
                                    
                                    q5 = np.arctan2(s5,c5) # theta5 result (rad)
                                    
                                    q6 = np.arctan2(R3e[1,1],R3e[1,2]) # theta6 result (rad)
                                    
                                    q4 = np.arctan2(R3e[2,0],R3e[0,0]) # theta4 result (rad)
                                    
                                    result.append([q1,q2,q3,q4,q5,q6])
        return result            

                