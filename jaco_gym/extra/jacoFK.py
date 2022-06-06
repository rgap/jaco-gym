"""Forward Kinematics for Jaco2 robotics manipulator
---------------------------------------------------------
This file calculates the transformation matrix and the
position of the end-effector w.r.t the base of the robot
Input:
        qjaco: a numpy array including 6 joints angles
Output:
        T: transformation matrix between the end-effector and the base
        E: position of the end-effector
Example:
        qjaco1 = np.array([[261.74, 171.01, 67.6, 152.93, 37.12, 6.91]])
        jacoFK(qjaco1)
---------------------------------------------------------
        Code: Reza Ahmadzadeh, Roshni Kaushik (IRIM-2016)
        email: reza.ahmadzadeh@gatech.edu
---------------------------------------------------------
"""
import numpy as np

class JacoFK:

    def __init__(self):
        self.D1 = 0.2755   # base to elbow
        self.D2 = 0.4100   # arm length
        self.D3 = 0.2073   # front arm length
        self.D4 = 0.0741   # first wrist length
        self.D5 = 0.0741   # second wrist length
        self.D6 = 0.1600   # wrist to center of the hand
        self.e2 = 0.0098   # joint 3-4 lateral offset
        pass

    def change_env(self, base_to_elbow, arm_length, front_arm_length):
        self.D1 = base_to_elbow
        self.D2 = arm_length
        self.D3 = front_arm_length
        # self.D4 = 0.2073
        # self.D5 = 0.0741
        # self.D6 = 0.1600
        # self.e2 = 0.0098

    def gripper_position(self, qjaco, verbose=False):
        s = qjaco.shape
        if s[0] != 1 or s[1] != 6:
            print('The input vector must be size 1x6')
            return
        q = qjaco + np.array([[0, -90, 90, 0, -180, 90]])
        q[0, 0] *= -1
        q = q*np.pi/180.0
        T = self.forward_kinematics(q)
        E = T[0:3, 3]
        if verbose:
            print('T:')
            print(T)
            print('End Effector:')
            print(E)
        return(E)


    def forward_kinematics(self, q):
        num_joints = q.shape[1]
        T = np.eye(4)
        for ii in range(num_joints):
            A = self.calculateA(q[0, ii], ii)
            T = np.dot(T, A)
        return T


    def calculateA(self, q, n):

        aa = 30*np.pi/180
        ca = np.cos(aa)
        sa = np.sin(aa)
        c2a = np.cos(2*aa)
        s2a = np.sin(2*aa)
        d4b = self.D3 + sa/s2a*self.D4
        d5b = (sa/s2a*self.D4 + sa/s2a*self.D5)
        d6b = (sa/s2a*self.D5 + self.D6)

        alpha = np.array([[np.pi/2, np.pi, np.pi/2, 2*aa, 2*aa, np.pi]])
        a = np.array([[0.0, self.D2, 0.0, 0.0, 0.0, 0.0]])
        d = np.array([[self.D1, 0.0, -self.e2, -d4b, -d5b, -d6b]])

        A = np.array([[np.cos(q), -np.sin(q)*np.cos(alpha[0, n]), np.sin(q)*np.sin(alpha[0, n]), a[0, n] * np.cos(q)],
                      [np.sin(q), np.cos(q)*np.cos(alpha[0, n]), -np.cos(q)*np.sin(alpha[0, n]), a[0, n] * np.sin(q)],
                      [0.0, np.sin(alpha[0, n]), np.cos(alpha[0, n]), d[0, n]], [0.0, 0.0, 0.0, 1.0]])
        return A

if __name__ == "__main__":
    # pass
    jacofk = JacoFK()
    # x_target = 0.5
    # y_target = 0.1
    # z_target = 0.08

    angles = [185.835,  240.,     100.,     280.,      74.0525,  90.6265]
    angles = jacofk.gripper_position(np.array([angles]))

    print('tip position:::', angles)
