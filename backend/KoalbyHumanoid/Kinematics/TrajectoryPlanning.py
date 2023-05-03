import math
import time

import numpy

from collections import namedtuple


class TrajPlanner:

    def generate_cubic_traj_coefficients(self, t0, tf, p0, pf, v0,
                                         vf):  # calculates cubic trajectory FOR A SINGLE JOINT, MUST IMPLEMENT IT JOINT BY JOINT

        # constraint equations
        # p0 = a + b*t0 + c*(t0**2) + d*(t0**3)
        # v0 = b + 2*c*t0 + 3*d*(t0**2)
        # pf = a + b * tf + c * (tf ** 2) + d * (tf ** 3)
        # vf = b + 2 * c * tf + 3 * d * (tf ** 2)
        # print([t0, tf, p0, pf, v0, vf])
        # set them up as matrices
        Mheight, Mwidth = 4, 4
        Mlist = [1, t0, t0 ** 2, t0 ** 3, 0, 1, 2 * t0, 3 * (t0 ** 2), 1, tf, tf ** 2, tf ** 3, 0, 1, 2 * tf,
                 3 * (tf * 2)]
        M = numpy.array(Mlist).reshape(Mheight, Mwidth)
        result = [p0, v0, pf, vf]  # might have to use numpy to make this
        Minverse = numpy.linalg.inv(M)
        coefficients = numpy.dot(Minverse, result)

        # print(coefficients)

        return coefficients  # should be [a, b, c, d]

    def generate_cubic_trajectory_joint_points(self, t0, tf, p0, pf, v0, vf):  # DONE FOR EACH JOINT
        trajPointsForJoint = [p0]
        currentCoefficients = self.generate_cubic_traj_coefficients(t0, tf, p0, pf, v0, vf)
        a = currentCoefficients[0]
        b = currentCoefficients[1]
        c = currentCoefficients[2]
        d = currentCoefficients[3]
        while t0 < tf:
            t0 += 0.1  # generate full position list, divide final time by number of positions to iterate through for full traj time
            currPos = a + b * t0 + c * (t0 ** 2) + d * (t0 ** 3)
            trajPointsForJoint.append(currPos)
        # print(trajPointsForJoint)
        return trajPointsForJoint  # [joint1value1, joint1value2, ...]

    def generate_full_cubic_traj_between_two_positions(self, t0, tf, positionList, v0,
                                                       vf):  # calls above function three times, one for each joint, then combines them into position list
        # print(positionList)
        finalPositionList = []
        position1 = positionList[0]
        position2 = positionList[1]
        for idx in range(0, len(position1)):  # for joint value in position
            # print(idx)
            # print(range(len(position)))
            # print(len(position))
            if (idx == len(position1)) or (idx == len(position2)):
                break
            # print(position[idx])
            # print(position[idx + 1])
            jointIterations = self.generate_cubic_trajectory_joint_points(t0, tf, position1[idx], position2[idx], v0,
                                                                          vf)
            # print(jointIterations)
            # print(jointIterations)
            finalPositionList.append(jointIterations)  # [[joint1values], [joint2values], joint3values]]
        # print(finalPositionList)
        usableFinalPositionList = self.make_into_usable_positions(
            finalPositionList)  # [[joint1value1, joint2value1, joint3value1], [...]]
        # print(usableFinalPositionList)
        return usableFinalPositionList

    def make_into_usable_positions(self,
                                   finalPositionList):  # takes in finalPositionList from above and converts it to list of positions
        useThisForIteratingThrough = []
        jointValues = []
        # print("Final Position List")
        # print(finalPositionList)
        # print(finalPositionList)
        for i in range(0, len(finalPositionList[0])):
            onePos = []
            for idx in range(0, len(finalPositionList)):
                onePos.append((finalPositionList[idx])[i])
            jointValues.append(onePos)
        # useThisForIteratingThrough.append(jointValues)
            # interMediateJointValues = finalPositionList[i]
            # jointValues.append(interMediateJointValues)
            #
            # print("Joint Values")
            # print(jointValues)
        # jointValues1 = finalPositionList[0]  # list of iterable values for a SINGLE JOINT
        # # print(jointValues1)
        # jointValues2 = finalPositionList[1]  # list of iterable values for a SINGLE JOINT
        # # print(jointValues2)
        # jointValues3 = finalPositionList[2]  # list of iterable values for a SINGLE JOINT
        #     jointValue = []
        # for i in range(0, len(jointValues[i])):  # for each value in list of joint values
        #     for idx in range(0, len(jointValues[0])):
        #
        #     # print(idx)
        #     jointValue.append((jointValues[i])[idx])
            # jointValue2 = jointValues2[idx]
            # jointValue3 = jointValues3[idx]
        # useThisForIteratingThrough.append(jointValue)  # converted to usable position to be fed into robot
        # print("useThisForIteratingThrough")
        # print(jointValues)
        return jointValues  # [[joint1value1, joint2value1, joint3value1], [...]]

    def convert_to_dictionary(self, keys, position):
        # print("Position")
        # print(position)
        positionDictionary = {}
        for i in range(0, len(position)):
            positionDictionary[keys[i]] = position[i]
        # positionDictionary = {keys[0]: position[0], keys[1]: position[1], keys[2]: position[2], 0: -20, 4: -20, 3: 20, 7: 40}    # IMPORTANT
        return positionDictionary  # {13: jointvalue, 14: jointvalue, 15: jointvalue}  this needs to be set to robot.motorPositionDict

    def execute_cubic_traj(self, positionList, keys, t0, tf, v0, vf):
        generatedPositionList = []
        # print("Position List")
        # print(positionList)
        for i in range(0, len(positionList) - 1):
            # if i == len(positionList - 1):
            #     break
            intermediatePositionList = [positionList[i], positionList[i + 1]]
            newPositionList = self.generate_full_cubic_traj_between_two_positions(t0, tf,
                                                                                  intermediatePositionList,
                                                                                  v0, vf)
            for onePosition in newPositionList:
                # print(newPositionList)  # [value1, value2, value3] in order of trajectory???
                generatedPositionList.append(onePosition)  # TEST TEST TEST
            # length of list is (trajTime/timeStep + 1) * len(positionList)
        # print(generatedPositionList)

        returnThisPositionDict = []
        for allPositions in generatedPositionList:  # there's two of these, each is the path from 1 point to another
            # print("AllPositions")
            # print(allPositions)

            positionDict = self.convert_to_dictionary(keys, allPositions)  # [v1, v2, v3]
            # print(positionDict)
            returnThisPositionDict.append(positionDict)

        return returnThisPositionDict
