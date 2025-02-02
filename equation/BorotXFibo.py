import numpy as np
from scipy.spatial.transform import Rotation as R

class FiboX_Borot:
    def __init__(self):
        ##### set parameter of robot 6 DOF (mm)
        self.d3 = 14.86
        self.l3 = 160
        self.l45 = 35.42+113.56
        self.l2 = 37.5
        self.d2 = 50.6
        self.d1 = 39.8
        self.l1 = 167.6
        self.le = 10
        self.l6 = 19
    # === Define DH Parameters ===
    def dh_transform(self, a, alpha, d, theta, theta1):
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
        Rz1 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                       [np.sin(theta1), np.cos(theta1), 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        return Tx @ Rx @ Tz @ Rz @ Rz1
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
            T = T @ self.dh_transform(a, alpha, d, theta, theta1)
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
        result = None
        q1,q2,q3,q4,q5,q6 = [None, None, None, None, None, None]
        flat = [False, False, False]
        rpy = np.radians([roll, pitch, yaw])
        R0e = R.from_euler('xyz', rpy).as_matrix()
        p0e = np.array([[xe], [ye], [ze]]) 
        de = np.array([[0], 
                    [0],
                    [self.le]])
        wrist = p0e - R0e @ de
        xw,yw,zw = [wrist[0,0], wrist[1,0], wrist[2,0]]
        #### Inverse position kinematic 0 - wrist => q1, q2, q3
        k2 = self.d2 - self.d1
        check_sqrt = xw**2 + yw**2 - k2**2
        if check_sqrt > 0:
            flat[0] = True
        # else:
            # print("Your position is impossible")
        if flat[0]:
            a = B[0]*np.sqrt(xw**2 + yw**2 - k2**2) - self.l2
            b = zw - self.l1
            k = 2*self.d3*self.l3
            m = 2*self.l45*self.l3
            n = a**2 + b**2 - self.l45**2 - self.d3**2 - self.l3**2
            
            root3 = np.roots([k**2+m, -2*n*k, n**2-m])
            is_real = np.all(np.isreal(root3))
            q3s = []
            if is_real:
                flat[1] = True
            # else:
                # print("Your position is impossible")
            if flat[1]:
                for i in root3:
                    q3 = np.arctan2(1-i**2,i)  
                    # print(q3)
                    q3s.append(q3) 
                    if B[1] == 0:
                        a1 = self.l45*np.cos(q3s[0]) - self.d3*np.sin(q3s[0])
                        a2 = self.l45*np.sin(q3s[0]) + self.d3*np.cos(q3s[0]) + self.l3
                        a3 = self.l45*np.sin(q3s[0]) + self.d3*np.cos(q3s[0]) + self.l3
                        a4 = self.l45*np.cos(q3s[0]) - self.d3*np.sin(q3s[0])
                    elif B[1] == 1:
                        a1 = self.l45*np.cos(q3s[1]) - self.d3*np.sin(q3s[1])
                        a2 = self.l45*np.sin(q3s[1]) + self.d3*np.cos(q3s[1]) + self.l3
                        a3 = self.l45*np.sin(q3s[1]) + self.d3*np.cos(q3s[1]) + self.l3
                        a4 = self.l45*np.cos(q3s[1]) - self.d3*np.sin(q3s[1])
                        
                    A = np.array([[-a2, a1], 
                                [a4, a3]])
                    ab = np.array([[a],
                                [b]])
                    
                    if np.linalg.det(A) != 0:
                        flat[2] = True  
                    # else:
                    #     print("Your position is impossible")
                    if flat[2]:
                        A_inv = np.linalg.inv(A)  
                        sc = A_inv @ ab 
                        q2 = np.arctan2(sc[0][0], sc[1][0])
                        k1 = self.l45*np.cos(q2+q3) - self.d3*np.sin(q2+q3) + self.l2 - self.l3*np.sin(q2)
                        gamma = np.arctan2(k2,k1)
                        q1 = np.arctan2(yw,xw) - gamma
                        # q1 = np.degrees(q1) 
                        # q2 = np.degrees(q2) 
                        # q3 = np.degrees(q3) 
                        
                        #### Forward orientation matrix => R03
                        R03 = np.array([[-np.cos(q1)*np.sin(q2+q3), -np.cos(q1)*np.cos(q2+q3), np.sin(q1)],
                            [-np.sin(q1)*np.sin(q2+q3), -np.sin(q1)*np.cos(q2+q3), -np.cos(q1)],
                            [np.cos(q2+q3), -np.sin(q2+q3), 0]])
                        R03_T = R03.T
                        #### Result of orientation e เทียบ 3
                        Rr = np.dot(R03_T, R0e)

                        ##### Inverse orientaton of last three joint
                        r11 = Rr[0,0]
                        r21 = Rr[1,0]
                        r22 = Rr[1,1]
                        r23 = Rr[1,2]
                        r31 = Rr[2,0]

                        s5 = B[2]*np.sqrt(r23**2 + r22**2)
                        c5 = -r21

                        q5 = np.arctan2(s5,c5)

                        s6 = r22
                        c6 = r23

                        q6 = np.arctan2(s6,c6)

                        c4 = r11
                        s4 = r31

                        q4 = np.arctan2(s4,c4)
                        result = [q1,q2,q3,q4,q5,q6]
        return result
                        # print(q1,q2,q3,q4,q5,q6)

        