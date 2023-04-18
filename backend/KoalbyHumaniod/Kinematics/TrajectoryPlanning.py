import math
import time

import numpy

from collections import namedtuple


class TrajPlanner:

    def generate_cubic_traj_coefficients(self, t0, tf, p0, pf, v0,
                                         vf):
        """
        Calculates cubic trajectory FOR A SINGLE JOINT, MUST IMPLEMENT IT JOINT BY JOINT

        :param t0: Initial time
        :param tf: Final desired time
        :param p0: Initial position
        :param pf: Final desired position
        :param v0: Initial velocity
        :param vf: Final desired velocity
        :return: A vector of the coefficients of the cubic trajectory equation
        """

        # constraint equations
        # p0 = a + b*t0 + c*(t0**2) + d*(t0**3)
        # v0 = b + 2*c*t0 + 3*d*(t0**2)
        # pf = a + b * tf + c * (tf ** 2) + d * (tf ** 3)
        # vf = b + 2 * c * tf + 3 * d * (tf ** 2)

        # Set them up as matrices
        Mheight, Mwidth = 4, 4
        Mlist = [1, t0, t0 ** 2, t0 ** 3, 0, 1, 2 * t0, 3 * (t0 ** 2), 1, tf, tf ** 2, tf ** 3, 0, 1, 2 * tf,  # cubic traj. function
                 3 * (tf * 2)]
        M = numpy.array(Mlist).reshape(Mheight, Mwidth)
        result = [p0, v0, pf, vf]  # vector of cubic trajectory function solutions
        Minverse = numpy.linalg.inv(M)  # inverse of M
        coefficients = numpy.dot(Minverse, result)  # dot product of result and Minverse to get vector of coefficients

        # print(coefficients)

        return coefficients  # vector of coefficients

    def generate_cubic_traj_for_joint_in_full_position(self, t0, tf, p0, pf, v0, vf):
        """
        Generates intermediate points between A SINGLE JOINT
        :param t0: Initial time
        :param tf: Final desired time
        :param p0: Initial position
        :param pf: Final desired position
        :param v0: Initial Position
        :param vf: Final desired position
        :return: [joint1val1, joint1val2, joint1val3, ...]
        """
        trajPointsForJoint = [p0]  # initial position
        currentCoefficients = self.generate_cubic_traj_coefficients(t0, tf, p0, pf, v0, vf)  # gets cubic traj. coefficients
        # each cubic traj coefficient
        a = currentCoefficients[0]
        b = currentCoefficients[1]
        c = currentCoefficients[2]
        d = currentCoefficients[3]
        while t0 < tf:
            t0 += 0.1  # generate full position list, divide final time by number of positions to iterate through for full traj time
            currPos = a + b * t0 + c * (t0 ** 2) + d * (t0 ** 3)
            trajPointsForJoint.append(currPos)
        # print(trajPointsForJoint)
        return trajPointsForJoint

    def generate_full_cubic_traj_between_two_full_positions(self, t0, tf, positionList, v0,
                                                       vf):
        """
        Goes through each of the three joints in the position and generates the intermediate values between each
        :param t0: Initial time
        :param tf: Final desired time
        :param positionList: List of initial and final positions
        :param v0: Initial velocity
        :param vf: Final desired velocity
        :return: List of positions to iterate through (not in correct dictionary form)
        """
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
            jointIterations = self.generate_cubic_traj_for_joint_in_full_position(t0, tf, position1[idx], position2[idx], v0,
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
        """
        Converts the input list to have a list of each position rather than a list for every position of a SINGLE JOINT
        :param finalPositionList: [[list of joint 1 positions], [joint 2 positions], [joint 3 positions], ...]
        :return: Values arranged by position [[pos1val1, pos1val2, pos1val3], [...], ...]
        """
        jointValues = []
        # print("Final Position List")
        # print(finalPositionList)
        # print(finalPositionList)
        for i in range(0, len(finalPositionList[0])):  # for each list of values for one joint
            onePos = []
            for idx in range(0, len(finalPositionList)):  # for each joint in that list
                onePos.append((finalPositionList[idx])[i])
            jointValues.append(onePos) # add [pos1val1, pos1val2, pos1val3] to jointValues, do it for every position
        return jointValues

    def convert_to_dictionary(self, keys, position):
        """
        Converts the list of traj. positions to a usable dictionary form to be sent to the robot
        :param keys: List of keys for motor joints depending on the leg chosen
        :param position: The position being converted
        :return: The input position as a dictionary
        """
        # print("Position")
        # print(position)
        positionDictionary = {}
        for i in range(0, len(position)):
            positionDictionary[keys[i]] = position[i]  # sets each key to its respective angle position
        return positionDictionary  # {13: jointvalue, 14: jointvalue, 15: jointvalue}  this needs to be set to robot.motorPositionDict

    def execute_cubic_traj(self, positionList, keys, t0, tf, v0, vf):
        """
        Calculates and executes trajectory movement
        :param positionList: List of the initial and final position [[initialjoint1, initialjoint2, initialjoint3], [finaljoint1, finaljoint2, finaljoint3]]
        :param keys:  List of keys of leg motors
        :param t0: Initial time
        :param tf: Final desired time
        :param v0: Initial velocity
        :param vf: Final desired velocity
        :return: Full list of positions along trajectory to be fed to the robot
        """
        newPositionList = []
        # print("Position List")
        # print(positionList)
        for i in range(0, len(positionList) - 1):  # this section generates full cubic traj. in non-dictionary form
            intermediatePositionList = [positionList[i], positionList[i + 1]]
            newPositionList = self.generate_full_cubic_traj_between_two_full_positions(t0, tf,
                                                                                  intermediatePositionList,
                                                                                  v0, vf)
            # for onePosition in newPositionList:  # this line and line after it seem unnecessary, commenting out just in case I'm wrong
            #     generatedPositionList.append(onePosition)
            # length of list is (trajTime/timeStep + 1) * len(positionList)

        returnThisPositionDict = []
        for allPositions in newPositionList:
            # print("AllPositions")
            # print(allPositions)

            positionDict = self.convert_to_dictionary(keys, allPositions)  # [v1, v2, v3] to [{key1, value1}, {key2, value2}, {key3, value3}]
            # print(positionDict)
            returnThisPositionDict.append(positionDict)

        return returnThisPositionDict  # final trajectory in dictionary form to be sent to the robot
