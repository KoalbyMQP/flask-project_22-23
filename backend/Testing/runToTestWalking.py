import csv
import sys

from backend.KoalbyHumaniod.Kinematics.TrajectoryPlanning import TrajPlanner
from backend.KoalbyHumaniod.Robot import SimRobot, Robot
from backend.Primitives.MovementManager import play_motion_kinematics
from backend.simulation import sim as vrep


class Walker:
    """
    Walker Class Definition
    """

    def __init__(self, isWalking):
        self.client_id = None
        self.isWalking = isWalking

    def init_sim(self):
        """
        Initializes the simulated robot
        """
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if self.client_id != -1:  # TODO: Fix duplicate code
            print("Connected to remote API server")
        else:
            sys.exit("Not connected to remote API server")

    def play(self, legChoice, tf, robot):
        """
        Makes the robot start walking when the appropriate UI button is pressed and will keep walking until the button
        is pressed again
        :param legChoice: The starting leg of the walk cycle
        :param tf: Full desired trajectory time
        :param robot: Robot instance
        :return: Calls play_motion_kinematics to iterate through a list of positions generated
        by trajectory planning functionality
        """
        numTimes = 0
        rightLegPositions = []
        leftLegPositions = []
        rightLegPositions2 = []
        leftLegPositions2 = []
        while numTimes < 2:
            # TODO: make this not an absolute filepath
            if numTimes == 0:
                # opens pre-generated CSV file and creates a list of trajectory positions for each leg,
                # one for when the leg is stepping and one for when the leg isn't stepping, for both legs
                with open(
                        "C:/Users/josh3/Documents/GitHub/flask-project/backend/Primitives/poses/leftWalk") as f: # left leg taking step
                    csvRecordedPoses = [{k: int(v) for k, v in row.items()}
                                        for row in
                                        csv.DictReader(f,
                                                       skipinitialspace=True)]  # parses selected csv file into list of poses
            if numTimes == 1:
                with open(
                        "C:/Users/josh3/Documents/GitHub/flask-project/backend/Primitives/poses/rightWalk") as f: # right leg taking step
                    csvRecordedPoses = [{k: int(v) for k, v in row.items()}
                                        for row in
                                        csv.DictReader(f,
                                                       skipinitialspace=True)]  # parses selected csv file into list of poses
            for poseMotorPositionsDict in csvRecordedPoses:  # for each position in the list of recorded positions
                intermediatePositionLeft = []
                intermediatePositionRight = []
                dummy_position = poseMotorPositionsDict
                for key in dummy_position.keys():  # add each position to the lists representing the left or right leg depending on motor ID
                    if key == '13' or key == '14' or key == '15':  # if position for left leg
                        intermediatePositionLeft.append(dummy_position[key])
                    if key == '18' or key == '19' or key == '20':  # if position for right leg
                        intermediatePositionRight.append(dummy_position[key])
                if numTimes == 0:
                    leftLegPositions.append(intermediatePositionLeft)  # step with left leg
                    rightLegPositions.append(intermediatePositionRight)
                if numTimes == 1:
                    leftLegPositions2.append(intermediatePositionLeft)  # step with right leg
                    rightLegPositions2.append(intermediatePositionRight)
            numTimes = numTimes + 1

        # print("Right then left 1 then 2")
        # print(rightLegPositions)
        # print(leftLegPositions)
        # print(rightLegPositions2)
        # print(leftLegPositions2)

        if legChoice == 1:  # if left leg chosen by user
            legChoice = 2  # makes it so next step is with the right leg
            positionList = leftLegPositions
            positionList2 = rightLegPositions
            keys = [13, 14, 15]
            keys2 = [18, 19, 20]
        else:  # if right leg chosen by user
            legChoice = 1  # makes it so next step is with the left leg
            positionList = rightLegPositions2
            positionList2 = leftLegPositions2
            keys = [18, 19, 20]
            keys2 = [13, 14, 15]

        t0 = 0  # initial time set to 0
        v0 = 0  # initial velocity set to 0
        vf = 0  # final velocity set to 0

        while self.isWalking:  # while the robot is walking
            # print("New Step")
            iterateThroughThis = trajPlanner.execute_cubic_traj(positionList, keys, t0, tf, v0, vf)  # positions for one leg to iterate through
            iterateThroughThis2 = trajPlanner.execute_cubic_traj(positionList2, keys2, t0, tf, v0, vf)  # positions for other leg to iterate through
            iterateThroughThisFinal = []

            for i in range(len(iterateThroughThis) - 1):  # combines the two lists above into a single trajectory plan
                iterateThroughThisFinal.append(iterateThroughThis[i])
                iterateThroughThisFinal.append(iterateThroughThis2[i])

            print("iterateThroughThisFinal")
            print(iterateThroughThisFinal)
            play_motion_kinematics(robot, iterateThroughThis)

            if legChoice == 1:  # if left leg is current leg
                legChoice = 2  # makes it so next step is with the right leg
                positionList = leftLegPositions
                positionList2 = rightLegPositions
                keys = [13, 14, 15]
                keys2 = [18, 19, 20]
            else:  # if right leg current leg
                legChoice = 1  # makes it so next step is with the left leg
                positionList = rightLegPositions2
                positionList2 = leftLegPositions2
                keys = [18, 19, 20]
                keys2 = [13, 14, 15]


if __name__ == "__main__":
    walker = Walker(True)
    simFlag = float(input("Enter 1 if in simulation or two if in real-world:"))
    if simFlag == 2:
        robot = Robot(walker.client_id)
    else:
        walker.init_sim()  # initializes the robot in simulation
        robot = SimRobot(walker.client_id)
        handle = vrep.simxGetObjectHandle(walker.client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]
        vrep.simxSetObjectFloatParameter(walker.client_id, handle, vrep.sim_shapefloatparam_mass, 5,  # gives sim cart a mass of 5 (unsure of units)
                                         vrep.simx_opmode_blocking)

    trajPlanner = TrajPlanner()  # init instance of trajectory planner
    legChoice = float(input("Enter 1 to move left leg or 2 to move right leg:"))
    tf = float(input("Enter trajectory time (seconds):"))
    walker.play(legChoice, tf, robot)
    walker.isWalking = bool(input("False to stop walking"))


def play_motion_kinematics(robot, dictList):
    # TODO: integrate this into play_motion?
    """
        iterates through list of recorded poses of entire robot,
        holding each pose for defined pose time.
        """

    for poseMotorPositionsDict in dictList:  # for each pose in the list of recorded poses
        modifyThisPositionDict = poseMotorPositionsDict
        print(modifyThisPositionDict)
        robot.update_motors(5000, modifyThisPositionDict)  # updates motors with new positions
