import math
import numpy as np

dims = np.array([76, 73, 119, 155, 11])
DHTable = np.array([[math.pi/2, dims[0], dims[1], -math.pi/2],
                    [0, dims[2], 0, math.pi/2],
                    [0, 0, 0, -math.pi/2],
                    [-math.pi/2, dims[3], dims[4], -math.pi/2]])

#  Given a row from the DH table, return the corresponding A matrix 
#  (Homogeneous Transformation Matrix).
#  row [1x4 double] - one row of the DH table to get the A matrix
def getDHRowMat(row):
    theta = row[0]
    d = row[1]
    a = row[2]
    alpha = row[3]

    A = np.zeros([4, 4])

    #  Fill out A matrix based on DH conventions
    A[0,:] = [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)]
    A[1,:] = [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)]
    A[2,:] = [0, math.sin(alpha), math.cos(alpha), d]
    A[3,:] = [0, 0, 0, 1]
    return A

# Given the 4 joint angles of each joint, return the 4 A matrices
# jointAngles [1x4 double] - The joint angles of the robot arm in
# rad
# returns: 4x4x4 matrix: [A0 A1 A2 A3]
def getIntMat(jointAngles):
    AMat = np.zeros([4, 4, 4])

    # copy DH table and fill in with actual joint angles
    dhTable = DHTable
    dhTable[:,0] = dhTable[:,0] + np.transpose(jointAngles)

    # Calculate each A matrix from corresponding row of DH table
    for i in range(4):
        AMat[:,:,i] = getDHRowMat(dhTable[i,:])
    return AMat

# Given the 4 joint angles of each joint, return the 4 T matrices
# (transform from the joint i to the base)
# jointAngles [1x4 double] - The joint angles of the robot arm in
# rad
# returns: 4x4x4 matrix: [T0_1 T0_2 T0_3 T0_4]
def getAccMat(jointAngles):
    TMat = np.zeros([4, 4, 4])

    # Get A matrices to multiply together
    AMat = getIntMat(jointAngles)
    
    # T_1^0 = A_1
    TMat[:,:,1] = AMat[:,:,1]
    # post-multiply A matrices to get each T_i^0 matrix
    for i in range(1, 4):
        TMat[:,:,i] = np.matmul(TMat[:,:,i-1], AMat[:,:,i])
    return TMat

# Given the 4 joint angles of each joint, return T0_4
# (Transformation from end effector frame to base frame)
# jointAngles [1x4 double] - The joint angles of the robot arm in
# rad
# returns: 4x4 matrix: T0_4
def getFK(jointAngles):
    # Get A matrices to multiply together
    AMat = getIntMat(jointAngles)

    # T_1^0 = A_1
    T = AMat[:,:,0]
    # post-multiply all A matrices in order to get in T_4^0
    for i in range(1,4):
        T = np.matmul(T, AMat[:,:,i])
    return T

print(getFK([math.pi,0,0,0]))