from scipy.optimize import fsolve
from scipy import io
import numpy as np
from math import cos
from math import sin
from naoqi import ALProxy
import time

def main(robotIP):

    # ��ʼ��
    JointAngles = {}
    JointNames = ['LAnklePitch', 'LAnkleRoll', 'LKneePitch', 'LHipPitch', 'LHipRoll', 'LHipYawPitch',
                  'RAnklePitch', 'RAnkleRoll', 'RKneePitch', 'RHipPitch', 'RHipRoll', 'RHipYawPitch']
    for name in JointNames:
        JointAngles[name] = []
        
    # x1,y1,z1Ϊ������ݣ�x2,y2,z2Ϊ�ҽ�����    
    data = io.loadmat('lift_foot.mat')
    x1 = data['x1'][0]
    y1 = data['y1'][0]
    z1 = data['z1'][0]
    x2 = data['x2'][0]
    y2 = data['y2'][0]
    z2 = data['z2'][0]
    
    # �������ò��Ų��Բ�̬�Ĵ���
    stepNum = 1

    length = len(x1)
    print length
    
    def leftfun(p):
        l1 = 45.19
        l2 = 102.99
        l3 = 100
        l4 = 50

        theta1 = float(p[0])
        theta2 = float(p[1])
        theta4 = float(p[2])

        return [
        l1*(sin(theta1)*(cos(theta1 + theta4)*cos(theta4) + sin(theta1 + theta4)*cos(theta2)*sin(theta4)) - 
        cos(theta1)*(cos(theta2)*(sin(theta1 + theta4)*cos(theta4) - cos(theta1 + theta4)*cos(theta2)*sin(theta4)) - 
        sin(theta2)**2*sin(theta4))) - l2*(sin(theta1 + theta4)*cos(theta4) - cos(theta1 + theta4)*cos(theta2)*sin(theta4)) + 
        l3*cos(theta2)*sin(theta4) - X,

        l3*sin(theta2) - l1*(cos(theta1)*(cos(theta2)*sin(theta2) - cos(theta1 + theta4)*cos(theta2)*sin(theta2)) - 
        sin(theta1 + theta4)*sin(theta1)*sin(theta2)) - l4 + l2*cos(theta1 + theta4)*sin(theta2) - Y,

        l4 + l2*(sin(theta1 + theta4)*sin(theta4) + cos(theta1 + theta4)*cos(theta2)*cos(theta4)) - 
        l1*(sin(theta1)*(cos(theta1 + theta4)*sin(theta4) - sin(theta1 + theta4)*cos(theta2)*cos(theta4)) - 
        cos(theta1)*(cos(theta4)*sin(theta2)**2 + cos(theta2)*(sin(theta1 + theta4)*sin(theta4) + 
        cos(theta1 + theta4)*cos(theta2)*cos(theta4)))) + l3*cos(theta2)*cos(theta4) - Z
        ]

    def rightfun(p):
        l1 = 45.19
        l2 = 102.99
        l3 = 100
        l4 = 50

        theta1 = float(p[0])
        theta2 = float(p[1])
        theta4 = float(p[2])

        return [
        l1*(sin(theta1)*(cos(theta1 + theta4)*cos(theta4) + sin(theta1 + theta4)*cos(theta2)*sin(theta4)) - 
        cos(theta1)*(cos(theta2)*(sin(theta1 + theta4)*cos(theta4) - cos(theta1 + theta4)*cos(theta2)*sin(theta4)) - 
        sin(theta2)**2*sin(theta4))) - l2*(sin(theta1 + theta4)*cos(theta4) - cos(theta1 + theta4)*cos(theta2)*sin(theta4)) + 
        l3*cos(theta2)*sin(theta4) - X,

        l4 - l1*(cos(theta1)*(cos(theta2)*sin(theta2) - cos(theta1 + theta4)*cos(theta2)*sin(theta2)) - 
        sin(theta1 + theta4)*sin(theta1)*sin(theta2)) + l3*sin(theta2) + l2*cos(theta1 + theta4)*sin(theta2) - Y,

        l4 + l2*(sin(theta1 + theta4)*sin(theta4) + cos(theta1 + theta4)*cos(theta2)*cos(theta4)) - 
        l1*(sin(theta1)*(cos(theta1 + theta4)*sin(theta4) - sin(theta1 + theta4)*cos(theta2)*cos(theta4)) - 
        cos(theta1)*(cos(theta4)*sin(theta2)**2 + cos(theta2)*(sin(theta1 + theta4)*sin(theta4) + 
        cos(theta1 + theta4)*cos(theta2)*cos(theta4)))) + l3*cos(theta2)*cos(theta4) - Z
        ]

     # �������ҽŵ�����
    for i in range(length):
        X = x1[i]
        Y = y1[i]
        Z = z1[i]
        result = fsolve(leftfun, [0,0,0])
        theta1 = result[0]
        theta2 = result[1]
        theta4 = result[2]
        theta3 = -theta1-theta4
        theta5 = -theta2
        JointAngles['LAnklePitch'].append(-theta1)
        JointAngles['LAnkleRoll'].append(theta2)
        JointAngles['LKneePitch'].append(-theta3)
        JointAngles['LHipPitch'].append(-theta4)
        JointAngles['LHipRoll'].append(theta5)
        JointAngles['LHipYawPitch'].append(0.0)
        print 'left' + str(i)
    for i in range(length):
        X = x2[i]
        Y = y2[i]
        Z = z2[i]
        result = fsolve(rightfun, [0,0,0])
        theta1 = result[0]
        theta2 = result[1]
        theta4 = result[2]
        theta3 = -theta1-theta4
        theta5 = -theta2
        JointAngles['RAnklePitch'].append(-theta1)
        JointAngles['RAnkleRoll'].append(theta2)
        JointAngles['RKneePitch'].append(-theta3)
        JointAngles['RHipPitch'].append(-theta4)
        JointAngles['RHipRoll'].append(theta5)
        JointAngles['RHipYawPitch'].append(0.0)
        print 'right' + str(i)
    print 'Compute Finish.'

    JointMat = {}
    for joint in JointNames:
        JointMat[joint] = np.mat(JointAngles[joint])
    io.savemat('joint.mat', JointMat)

    PORT = 9559
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    motionProxy.setStiffnesses("Body", 1.0)
    time.sleep(2)

    X = 0.5             # �����򳤶ȶ�
    Y = 0.0             # �������󳤶�
    Theta = 0.0         # ��ʱ��ת
    Frequency = 0.5     # ��·Ƶ��,0.0���,1.0���
    #motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    #time.sleep(2.5)
    #motionProxy.setWalkTargetVelocity(0.0, 0.0, 0.0, 0.0)
    motionProxy.setStiffnesses("Body", 1.0)
    postureProxy.goToPosture("StandInit", 0.5)
    time.sleep(1)
    
    #fractionMaxSpeed = 0.23
    fractionMaxSpeed = 0.8


    # ����n��
    for j in range(stepNum):
        for i in range(length):
            angles = []
            for name in JointNames:
                angles.append(JointAngles[name][i])
            # print angles
            motionProxy.setAngles(JointNames, angles, fractionMaxSpeed)
            time.sleep(1)
            
if __name__ == "__main__":
    # robotIp = "169.254.50.227"
    robotIp = "192.168.43.104"
    #robotIp = "10.180.44.244"
    main(robotIp)