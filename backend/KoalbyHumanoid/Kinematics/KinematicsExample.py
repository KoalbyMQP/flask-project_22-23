import math
import numpy

from collections import namedtuple

from Kinematics.KinematicsClass import Link, Chain

# testLink1 = Link(math.pi/2, 1, 1, -math.pi/2, 2)
testLink1 = Link(0, 18.7325, 0, -math.pi/2, 18.7325)  # cm
testMatrix1 = testLink1.get_transformation_matrix(0)
# testMatrix1 = testLink1.get_transformation_matrix(math.pi/2)
# print("Test Matrix 1")
# print(testMatrix1)

# testLink2 = Link(0, 2, 1, math.pi/2, 2)
testLink2 = Link(-math.pi/2, 0, 21.2725, 0, 21.2725)  # cm
testMatrix2 = testLink2.get_transformation_matrix(0)
# print("Test Matrix 2")
# print(testMatrix2)

# testLink3 = Link(-math.pi/2, 1, 2, 0, 2)
testLink3 = Link(math.pi/2, 0, 3.4925, 0, 3.4925)  # cm
testMatrix3 = testLink3.get_transformation_matrix(0)
# print("Test Matrix 3")
# print(testMatrix3)

linkList = [testLink1, testLink2, testLink3]

jointDegList = [math.pi/2, math.pi/2, math.pi/2]
testChain = Chain(linkList)
fkResult1, fkResult2 = testChain.forward_kinematics(jointDegList)
print("End Effector Transformation Matrix")
print(fkResult1)
# print("Intermediate Matrices")
# print(fkResult2)

# p = get_task_space(fkResult1)
# print(p)
q = testChain.inverse_kinematics_leg(fkResult1)
print("Joint Space Angles")
print(q)

print("Confirm math, result should equal fkResult1")
qConfirm, babe = testChain.forward_kinematics(q)
print(qConfirm)
