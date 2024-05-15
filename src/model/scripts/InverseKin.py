import math
import numpy as np

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32




class InverseKin(object):
    """autoencoder definition
    """
    def __init__(self, para):
        super(InverseKin, self).__init__()
        # self.pos = pos
        self.a = para[0]
        self.b = para[1]
        self.l1 = para[2]
        self.l2 = para[3]
        self.l3 = para[4]

    def position(self, P):

        th_initial = [0, 0]
        th = [0,0]

        phi1 = math.atan2(P[0], P[2])
        phi2 = math.atan2(-P[1] * math.cos(phi1), P[2])
        # phi[0] = math.atan2(P[0], P[2])
        # phi[1] = math.atan2(-P[1] * math.cos(phi[0]), P[2])
        # theta = np.array([2, 2])
        # position kin
        # phi1 = self.pos[0]
        # phi2 = self.pos[1]
        a = self.a
        b = self.b
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3

        E1 = 2 * a * l1 - (2 * l1 * l3 * math.cos(phi1))
        F1 = 2 * l1 * (b - l3 * math.sin(phi1))
        G1 = a**2 + b**2 + l1**2 + l3**2 - l2**2 - 2 * l3 * (a * math.cos(phi1) + b * math.sin(phi1))
        E2 = 2 * a * l1 - (2 * l1 * l3 * math.cos(phi2))
        F2 = -2 * l1 * (b + l3 * math.cos(phi1) * math.sin(phi2))
        G2 = a**2 + b**2 + l1**2 + l3**2 - l2**2 - 2 * l3 * (a * math.cos(phi2) - b * math.cos(phi1) * math.sin(phi2))

        t1_1 = (-F1 + math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
        t1_2 = (-F1 - math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
        t2_1 = (-F2 + math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
        t2_2 = (-F2 - math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
        theta = [[2 * math.atan(t1_1), 2 * math.atan(t1_2)], [2 * math.atan(t2_1), 2 * math.atan(t2_2)]]
        for i in range(2):
            error1 = math.fabs(theta[i][0]) - math.fabs(th_initial[i])
            error2 = math.fabs(theta[i][1]) - math.fabs(th_initial[i])
            if error1 < error2:
                th[i] = theta[i][0]
            else:
                th[i] = theta[i][1]

        return th


P = [-150, 150, 400]
a = 25.25
b = 85
l1 = 55
l2 = 85
l3 = 80.25
para = [a,b,l1,l2,l3]
# phi = [0,0]
# phi[0] = math.atan2(P[0], P[2])
# phi[1] = math.atan2(-P[1]*math.cos(phi[0]), P[2])
# print(phi)

kin = InverseKin(para=para)
theta = kin.position(P)


print(theta)

if __name__ == '__main__':

    P = [-150, 150, 400]
    a = 25.25
    b = 85
    l1 = 55
    l2 = 85
    l3 = 80.25
    para = [a,b,l1,l2,l3]
    # phi = [0,0]
    # phi[0] = math.atan2(P[0], P[2])
    # phi[1] = math.atan2(-P[1]*math.cos(phi[0]), P[2])
    # print(phi)

    kin = InverseKin(para=para)
    theta = kin.position(P)