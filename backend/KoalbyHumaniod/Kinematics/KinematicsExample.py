import math
import numpy

from collections import namedtuple

from KinematicsClass import Link, Chain
from backend.KoalbyHumaniod.Kinematics.TrajectoryPlanning import TrajPlanner

trajPlanner = TrajPlanner()

keys = [18, 19, 20]
t0 = 0  # initial time set to 0
v0 = 0  # initial velocity set to 0
vf = 0  # final velocity set to 0
tf = 2

iterateThroughThis = trajPlanner.execute_cubic_traj([[10, 30, -30], [0, -20, 0]], keys, t0, tf, v0, vf)
print("iterateThroughThis")
print(iterateThroughThis)

# # testLink1 = Link(math.pi/2, 1, 1, -math.pi/2, 2)  # simple link for testing
# testLink1 = Link(0, 18.7325, 0, -math.pi/2, 18.7325)  # DH params for Koalby thigh link in cm with test angle value (which may not be realistic)
# testMatrix1 = testLink1.get_transformation_matrix(0)  # forward kinematics transformation matrix for first link with rotation angle as input
# # print("Test Matrix 1")
# # print(testMatrix1)
#
# # testLink2 = Link(0, 2, 1, math.pi/2, 2)  # simple link for testing
# testLink2 = Link(-math.pi/2, 0, 21.2725, 0, 21.2725)  # DH params for Koalby shin link in cm with test angle value (which may not be realistic)
# testMatrix2 = testLink2.get_transformation_matrix(0)  # # forward kinematics transformation matrix for second link with rotation angle as input
# # print("Test Matrix 2")
# # print(testMatrix2)
#
# # testLink3 = Link(-math.pi/2, 1, 2, 0, 2)  # simple link for testing
# testLink3 = Link(math.pi/2, 0, 3.4925, 0, 3.4925)  # DH params for Koalby foot link in cm (vertical distance from ankle to foot bottom) with test angle value (which may not be realistic)
# testMatrix3 = testLink3.get_transformation_matrix(0)  # forward kinematics transformation matrix for third link with rotation angle as input
# # print("Test Matrix 3")
# # print(testMatrix3)
#
# linkList = [testLink1, testLink2, testLink3]  # list of Koalby's links from thigh to foot
#
# jointDegList = [math.pi/2, math.pi/2, math.pi/2]  # test rotation angles for each link in the chain (probably not realistic)
# testChain = Chain(linkList)  # kinematic chain object of Koalby's leg
# fkResult1, fkResult2 = testChain.forward_kinematics(jointDegList)  # forward kinematics fom Koalby's leg
# print("End Effector Transformation Matrix")
# print(fkResult1)
# # print("Intermediate Matrices")
# # print(fkResult2)
#
# # p = get_task_space(fkResult1)
# # print(p)
# q = testChain.inverse_kinematics_leg(fkResult1)  # inverse kinematics of Koalby's leg
# print("Joint Space Angles")
# print(q)
#
# print("Confirm math, result should equal fkResult1")
# qConfirm, babe = testChain.forward_kinematics(q)  # forward kinematics on inverse kinematics result for confirmation
# print(qConfirm)
