from scipy.optimize import fsolve
from scipy import io
import numpy as np
from math import cos
from math import sin
from naoqi import ALProxy
import time
PI = 3.1415926535

def computeXYZ(d):
    lmax = 45.0
    a1 = 1.0*(lmax/16)
    a2 = 1.0*(-lmax/48)
    a3 = 23.0

    x1 = []
    y1 = []
    z1 = []
    x2 = []
    y2 = []
    z2 = []
    xiuzheng1 = []
    xiuzheng2 = []
    adjust1 = []
    adjust2 = []

    # ����x1
    for i in range(16):
        x1.append(a2*(i+16)+6.6667)
    for i in range(16):
        x1.append(a2*32+a1*i+6.6667)
    for i in range(16):
        x1.append(a2*32+a1*16+a2*i+6.6667)
    for i in range(16):
        x1.append(a2*i+6.6667)
    # ����x2
    for i in range(48):
        x2.append(a1*16+a2*i-8*a1)
    for i in range(16):
        x2.append(a1*i-8*a1)
    # ����xiuzheng1
    for i in range(16):
        xiuzheng1.append(0)
    for i in range(16):
        xiuzheng1.append(-d*(i+1)/16)
    for i in range(16):
        xiuzheng1.append(-d*(16-i)/16)
    for i in range(16):
        xiuzheng1.append(0)
    # ����xiuzheng2
    for i in range(32):
        xiuzheng2.append(0)
    for i in range(16):
        xiuzheng2.append(d*(i+1)/16)
    for i in range(16):
        xiuzheng2.append(d*(16-i)/16)
    # ����y1
    for i in range(64):
        y1.append(-50+65*cos((i+1)*PI/32+PI/4)+xiuzheng1[i])
    # ����y2
    for i in range(64):
        y2.append(50+65*cos((i+1)*PI/32+PI/4)+xiuzheng2[i])
    # ����z1
    for i in range(64):
        z1.append(283)
    for i in range(16):
        z1[i+16] = z1[i+16] - 45*sin(PI/16*(i+1))
    # ����z2
    for i in range(64):
        z2.append(283)
    for i in range(16):
        z2[i+48] = z2[i+48] - 45*sin(PI/16*(i+1))
    # ����adjust1
    for i in range(64):
        adjust1.append(0)
        if i==14 or i==32:
            adjust1[i] = 0.02
        elif i==15 or i==31:
            adjust1[i] = 0.04
        elif i>15 and i<31:
            adjust1[i] = 0.05
    # ����adjust2
    for i in range(64):
        adjust2.append(0)
        if i==46 or i==64:
            adjust2[i] = 0.02
        elif i==47 or i==63:
            adjust2[i] = 0.04
        elif i>47 and i<63:
            adjust2[i] = 0.05

    return x1,y1,z1,x2,y2,z2,adjust1,adjust2     

def computeFirstXYZ(x1,y1,z1,x2,y2,z2):

    x11 = []
    y11 = []
    z11 = []
    x22 = []
    y22 = []
    z22 = []
    adjust11 = []
    adjust22 = []
    
    # ����x11
    for i in range(24):
        x11.append(x1[0])
    # ����x22
    for i in range(16):
        x22.append(x1[0])
    for i in range(8):
        x22.append(x1[0] + 1.0*(x2[0]-x1[0])*(i+1)/8)
    # ����y11
    for i in range(24):
        y11.append(y1[40+i])
    # ����y22
    for i in range(24):
        y22.append(y2[40+i])
    # ����z11
    for i in range(24):
        z11.append(283)
    # ����z22
    for i in range(16):
        z22.append(283)
    for i in range(8):
        z22.append(283-20*sin(PI/8*(i+1)))
    # ����adjust11
    for i in range(24):
        adjust11.append(0)
    # ����adjust22
    for i in range(24):
        adjust22.append(0)

    return x11,y11,z11,x22,y22,z22,adjust11,adjust22

def main(robotIP):
    
    # ��ʼ��
    d = -20
    x1,y1,z1,x2,y2,z2,adjust1,adjust2 = computeXYZ(d)
    first_x1,first_y1,first_z1,first_x2,first_y2,first_z2,first_adjust1,first_adjust2 = computeFirstXYZ(x1,y1,z1,x2,y2,z2)
    JointAngles = {}
    JointNames = ['LAnklePitch', 'LAnkleRoll', 'LKneePitch', 'LHipPitch', 'LHipRoll', 'LHipYawPitch',
                  'RAnklePitch', 'RAnkleRoll', 'RKneePitch', 'RHipPitch', 'RHipRoll', 'RHipYawPitch']
    for name in JointNames:
        JointAngles[name] = []

    # ��ʼ�벽������
##    data = io.loadmat('15.mat')
##    first_x1 = data['x11'][0]
##    first_y1 = data['y11'][0]
##    first_z1 = data['z11'][0]#(data['z11'][0]-data['z22'][0])*0.5+data['z22'][0]
##    first_x2 = data['x22'][0]
##    first_y2 = data['y22'][0]
##    first_z2 = data['z22'][0]

    # ���������ظ��������߼�����
    stepNum = 15
    first_length = len(first_x1)
    print first_length
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
        theta5 = float(-theta2-ADJUST)

        return [
        100*cos(theta5)*sin(theta4) - (4519*cos(theta1)*(cos(theta2)*(sin(theta1 + theta4)*cos(theta4)
        - cos(theta1 + theta4)*cos(theta5)*sin(theta4)) + sin(theta2)*sin(theta4)*sin(theta5)))/100 +
        (4519*sin(theta1)*(cos(theta1 + theta4)*cos(theta4) + sin(theta1 + theta4)*cos(theta5)*sin(theta4)))/100 -
        (10299*sin(theta1 + theta4)*cos(theta4))/100 + (10299*cos(theta1 + theta4)*cos(theta5)*sin(theta4))/100 - X,
        
        - 100*sin(theta5) - (4519*cos(theta1)*(cos(theta5)*sin(theta2) + cos(theta1 + theta4)*cos(theta2)*sin(theta5)))/100 -
        (10299*cos(theta1 + theta4)*sin(theta5))/100 - (4519*sin(theta1 + theta4)*sin(theta1)*sin(theta5))/100 - 50 - Y,
        
        (10299*sin(theta1 + theta4)*sin(theta4))/100 + 100*cos(theta4)*cos(theta5) -
        (4519*sin(theta1)*(cos(theta1 + theta4)*sin(theta4) - sin(theta1 + theta4)*cos(theta4)*cos(theta5)))/100 +
        (4519*cos(theta1)*(cos(theta2)*(sin(theta1 + theta4)*sin(theta4) + cos(theta1 + theta4)*cos(theta4)*cos(theta5)) -
        cos(theta4)*sin(theta2)*sin(theta5)))/100 + (10299*cos(theta1 + theta4)*cos(theta4)*cos(theta5))/100 + 50 - Z
        ]

    def rightfun(p):
        l1 = 45.19
        l2 = 102.99
        l3 = 100
        l4 = 50

        theta1 = float(p[0])
        theta2 = float(p[1])
        theta4 = float(p[2])
        theta5 = float(-theta2+ADJUST)

        return [
        100*cos(theta5)*sin(theta4) - (4519*cos(theta1)*(cos(theta2)*(sin(theta1 + theta4)*cos(theta4) -
        cos(theta1 + theta4)*cos(theta5)*sin(theta4)) + sin(theta2)*sin(theta4)*sin(theta5)))/100 +
        (4519*sin(theta1)*(cos(theta1 + theta4)*cos(theta4) + sin(theta1 + theta4)*cos(theta5)*sin(theta4)))/100 -
        (10299*sin(theta1 + theta4)*cos(theta4))/100 + (10299*cos(theta1 + theta4)*cos(theta5)*sin(theta4))/100 - X,
        
        50 - (4519*cos(theta1)*(cos(theta5)*sin(theta2) + cos(theta1 + theta4)*cos(theta2)*sin(theta5)))/100 -
        (10299*cos(theta1 + theta4)*sin(theta5))/100 - (4519*sin(theta1 + theta4)*sin(theta1)*sin(theta5))/100 - 100*sin(theta5) - Y,
        
        (10299*sin(theta1 + theta4)*sin(theta4))/100 + 100*cos(theta4)*cos(theta5) -
        (4519*sin(theta1)*(cos(theta1 + theta4)*sin(theta4) - sin(theta1 + theta4)*cos(theta4)*cos(theta5)))/100 +
        (4519*cos(theta1)*(cos(theta2)*(sin(theta1 + theta4)*sin(theta4) + cos(theta1 + theta4)*cos(theta4)*cos(theta5)) -
                           cos(theta4)*sin(theta2)*sin(theta5)))/100 + (10299*cos(theta1 + theta4)*cos(theta4)*cos(theta5))/100 + 50 - Z
        ]

    # ���㿪ʼ�벽����
    for i in range(first_length):
        X = first_x1[i]
        Y = first_y1[i]
        Z = first_z1[i]*0.95
        ADJUST = first_adjust1[i]*0
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
        print 'firstleft' + str(i)
    for i in range(first_length):
        X = first_x2[i]
        Y = first_y2[i]
        Z = first_z2[i]*0.95
        ADJUST = first_adjust2[i]*0
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
        print 'firstright' + str(i)
    # �������n������
    for i in range(length):
        X = x1[i]
        Y = y1[i]
        Z = z1[i]*0.95
        ADJUST = adjust1[i]*0
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
        Z = z2[i]*0.95
        ADJUST = adjust2[i]*0
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
    fractionMaxSpeed = 0.4
    # ��ʼ�벽
    for i in range(first_length):
        angles = []
        for name in JointNames:
            angles.append(JointAngles[name][i])
        # print angles
        motionProxy.setAngles(JointNames, angles, fractionMaxSpeed)
        time.sleep(0.12)
    # ����n��
    for j in range(stepNum):
        for i in range(first_length, first_length + length):
            angles = []
            for name in JointNames:
                angles.append(JointAngles[name][i])
            # print angles
            motionProxy.setAngles(JointNames, angles, fractionMaxSpeed)
            time.sleep(0.03)
    
if __name__ == "__main__":
    robotIp = "192.168.0.1"
    main(robotIp)


