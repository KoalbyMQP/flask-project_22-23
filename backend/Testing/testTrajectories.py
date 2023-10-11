import sys
import time
#sys.path.insert(0, "C:\\Users\\sbpen\\OneDrive\\Desktop\\flask-project-main\\flask-project-main\\flask-project-main")
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import RealRobot, SimRobot
from backend.Simulation import sim as vrep
from backend.LimbTrajectories.rightLegTraj import *
from backend.LimbTrajectories.jointVelocityControl import Joint

def setup():
    simulation_flag = int(input("Are you running the Simulation?? Please enter 1 for yes and 0 for no: "))
    if simulation_flag == 1:
        vrep.simxFinish(-1)  # just in case, close all opened connections
        client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if client_id != -1:
            print("Connected to remote API server")
        else:
            sys.exit("Not connected to remote API server")
        robot = SimRobot(client_id)  # inits sim robot
    else:  # inits real-world robot
        robot = RealRobot()
        client_id = -1
    return robot, client_id

def moveMotors():
    rightShoulderRotator.move(rightShoulderRotator.target)
    rightShoulderAbductor.move(rightShoulderAbductor.target)
    rightArmRotator.move(rightArmRotator.target)
    rightElbow.move(rightElbow.target)

    leftShoulderRotator.move(leftShoulderRotator.target)
    leftShoulderAbductor.move(leftShoulderAbductor.target)
    leftArmRotator.move(leftArmRotator.target)
    leftElbow.move(leftElbow.target)

    lowerTorsoFront2Back.move(lowerTorsoFront2Back.target)
    chestSide2Side.move(chestSide2Side.target)
    lowerTorsoSide2Side.move(lowerTorsoSide2Side.target)
    upperTorsoRotator.move(upperTorsoRotator.target)

    rightLegAbductor.move(rightLegAbductor.target)
    rightLegRotator.move(rightLegRotator.target)
    rightKick.move(rightKick.target)
    rightKnee.move(rightKnee.target)
    rightAnkle.move(rightAnkle.target)

    leftLegAbductor.move(leftLegAbductor.target)
    leftLegRotator.move(leftLegRotator.target)
    leftKick.move(leftKick.target)
    leftKnee.move(leftKnee.target)
    leftAnkle.move(leftAnkle.target)

    neckForward2Back.move(neckForward2Back.target)
    neckRotator.move(neckRotator.target)
    pass

robot, client_id = setup()
Joint.startJointStreaming(robot, client_id)

rightShoulderRotator = Joint(robot.motors[0], 1, 0, 10, client_id)
rightShoulderAbductor = Joint(robot.motors[1], 1, 0, 10, client_id)
rightArmRotator = Joint(robot.motors[2], 1, 0, 10, client_id)
rightElbow = Joint(robot.motors[3], 1, 0, 10, client_id)

leftShoulderRotator = Joint(robot.motors[4], 1, 0, 10, client_id)
leftShoulderAbductor = Joint(robot.motors[5], 1, 0, 10, client_id)
leftArmRotator = Joint(robot.motors[6], 1, 0, 10, client_id)
leftElbow = Joint(robot.motors[7], 1, 0, 10, client_id)

lowerTorsoFront2Back = Joint(robot.motors[8], 1, 0, 10, client_id)
chestSide2Side = Joint(robot.motors[9], 1, 0, 10, client_id)
lowerTorsoSide2Side = Joint(robot.motors[11], 8, 0, 10, client_id)
upperTorsoRotator = Joint(robot.motors[12], 1, 0, 10, client_id)

rightLegAbductor = Joint(robot.motors[13], 1, 0, 10, client_id)
rightLegRotator = Joint(robot.motors[14], 1, 0, 10, client_id)
rightKick = Joint(robot.motors[15], 1, 0, 10, client_id)
rightKnee = Joint(robot.motors[16], 1, 0, 10, client_id)
rightAnkle = Joint(robot.motors[17], 1, 0, 10, client_id)

leftLegAbductor = Joint(robot.motors[18], 1, 0, 10, client_id)
leftLegRotator = Joint(robot.motors[19], 1, 0, 10, client_id)
leftKick = Joint(robot.motors[20], 1, 0, 10, client_id)
leftKnee = Joint(robot.motors[21], 1, 0, 10, client_id)
leftAnkle = Joint(robot.motors[22], 1, 0, 10, client_id)

neckForward2Back = Joint(robot.motors[23], 1, 0, 10, client_id)
neckRotator = Joint(robot.motors[24], 1, 0, 10, client_id)

print("Joint objects are created")

print("Setup Complete")

while True:
    moveMotors()


"""res = vrep.simx_return_novalue_flag
while res != vrep.simx_return_ok:
    res, data = vrep.simxGetObjectPosition(client_id, robot.motors[15].handle, robot.motors[8].handle, vrep.simx_opmode_streaming)

#home: 0  -10  10
moveRightLeg(robot, client_id, -90, -90, -90, 4)"""