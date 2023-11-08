import modern_robotics as mr
import numpy as np
import math

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

#Twists of Joints
S1 = [1, 0, 0, 0, 67.02, -74.53]
M1 = [
    [1, 0, 0, -101.09],
    [0, 1, 0, 74.53],
    [0, 0, 1, 67.02],
    [1, 0, 0, 1],
]

S2 = [0, 0, 1, 72.44, 127.29, 0]
M2 = [
    [1, 0, 0, -127.29],
    [0, 1, 0, 72.44],
    [0, 0, 1, 62.05],
    [1, 0, 0, 1], 
]
S3 = [1, 0, 0, 0, 67.37, -71.22]
S4 = [0, 1, 0, -54.34, 0, 330.19]
S5 = [0, 1, 0, -54.34, 0, 330.19]
S6 = [0, 1, 0, -54.34, 0, 330.19]
sList = [S1, S2, S3, S4, S5, S6]
thetaList = [0, 0, 0, 0, 0, 0]
M = [
    [1, 0, 0, 101.09],
    [0, 1, 0, 74.53],
    [0, 0, 1, 67.02],
    [0, 0, 0, 1]
]

R = rodriguez(S1, math.pi)
p = ramirez(S1, math.pi)

theta1 = math.pi / 6
t1 = mr.FKinSpace(M1, [S1, S1, S1, S1, S1, S1], [0, 0, 0, 0, 0, 0])
print(t1)
t2 = mr.FKinSpace(M2, [S1, S2, S2, S2, S2, S2], [0, 0, 0, 0, 0, 0])
print(t2)
"""R2 = rodriguez(S2, 0)
p2 = ramirez(S2, 0)

T2 = makeTMatrix(R2, p2)

Tfinal = T * T2"""

#print(T * M1)

#print("T")
#print(T)
#print("rodriguez")
#print(rodriguez(S1, math.pi))
#print("ramirez")
#print(ramirez(S1, 0))