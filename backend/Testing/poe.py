import modern_robotics as mr
import numpy as np
import math
import sys
sys.path.append("./")
from backend.KoalbyHumanoid import Config, Robot

def rodriguez(twist, theta):
    # R = I + sin(theta) * [w] + (1-cos(theta)) * [w]^2
    I = np.eye(3)
    w = mr.VecToso3([twist[0], twist[1], twist[2]])
    w2 = np.matmul(w, w)
    R = I + math.sin(theta) * w + (1-math.cos(theta)) * w2
    return R

def ramirez(twist, theta):
    # p = (theta * I + (1 - cos(theta)) * [w] + (theta - sin(theta) * [w]^2) * v 
    I = np.eye(3)
    w = mr.VecToso3([twist[0], twist[1], twist[2]])
    w2 = np.matmul(w, w)
    v = np.vstack([twist[3], twist[4], twist[5]])
    p = np.matmul(theta * I + (1 - math.cos(theta)) * w + (theta - math.sin(theta) * w2), v)
    return p

def makeTMatrix(R, p):
    bottomRow = np.zeros([1,4])
    bottomRow[-1] = 1
    T = np.concatenate((np.concatenate((R,p),axis=1), bottomRow), axis=0)
    return T

def calcCoMtest(tList, massList):
    """will improve to use joint/link data from Config.py"""
    massSum = 618.15
    weightX = 0
    weightY = 618.15 * 71.83
    weightZ = 618.15 * 54.35
    for i in range(0, len(tList)):
        massSum += massList[i]
        weightX += tList[i][0,3] * massList[i]
        weightY += tList[i][1,3] * massList[i]
        weightZ += tList[i][2,3] * massList[i]
    return [weightX/massSum, weightY/massSum, weightZ/massSum]

def calcCom():
    Slist = []
    for link in Config.rightArmLinks:
        Slist.append(link[1][6])
        M = link[4]
        mr.FKinSpace(M, Slist,)


#Twists of Joints
S1 = [-1, 0, 0, 0, -76, 73]
M1 = [
    [1, 0, 0, -101.09],
    [0, 1, 0, 74.53],
    [0, 0, 1, 67.02],
    [1, 0, 0, 1],
]

S2 = [0, 0, -1, -73.05, -119.1, 0]
M2 = [
    [1, 0, 0, -127.29],
    [0, 1, 0, 72.44],
    [0, 0, 1, 62.05],
    [1, 0, 0, 1], 
]
S3 = [1, 0, 0, 0, 76.65, -72]
M3 = [
    [1, 0, 0, -229.68],
    [0, 1, 0, 71.22],
    [0, 0, 1, 67.37],
    [0, 0, 0, 1]
]
S4 = [0, 1, 0, -64.63, 0, 247.17]
M4 = [
    [1,0,0,-330.19],
    [0,1,0,73.82],
    [0,0,1,54.34],
    [0,0,0,1]
]

"""sList = [S1, S2, S3, S4]
thetaList = [0, 0, 0, 0, 0, 0]
M = [
    [1, 0, 0, 101.09],
    [0, 1, 0, 74.53],
    [0, 0, 1, 67.02],
    [0, 0, 0, 1]
]"""

theta1 = math.radians(0)
theta2 = math.radians(0)
theta3 = math.radians(0)
theta4 = math.radians(0)

t1 = mr.FKinSpace(M1, [S1], [theta1])
t2 = mr.FKinSpace(M2, [S1, S2], [theta1, theta2])
t3 = mr.FKinSpace(M3, [S1, S2, S3], [theta1, theta2, theta3])
t4 = mr.FKinSpace(M4, [S1, S2, S3, S4], [theta1, theta2, theta3, theta4])

tList = [t1, t2, t3, t4]
massList = [10.17, 71.23, 206.87, 235.84]
#print(calcCoM(tList, massList))
calcCom()

"""
CoMx = (0*618.15 + t1[0,3]*10.17 + t2[0,3]*71.23) / (10.17 + 71.23+618.15)
CoMy = (71.83*618.15 + t1[1,3]*10.17 + t2[1,3]*71.23) / (10.17 + 71.23+618.15)
CoMz = (54.35*616.15 + t1[2,3]*10.17 + t2[2,3]*71.23) / (10.17 + 71.23+618.15)
print(CoMx, CoMy, CoMz)"""