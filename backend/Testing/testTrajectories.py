import sys
import time
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import RealRobot, SimRobot, Joints
from backend.Simulation import sim as vrep
from backend.LimbTrajectories.rightLegTraj import *
import matplotlib.pyplot as plt
#from backend.LimbTrajectories.jointVelocityControl import Joint

def setup():
    #x = vrep.simxStartSimulation(0, vrep.simx_opmode_oneshot)
    simulation_flag = int(input("Are you running the Simulation? Please enter 1 for yes and 0 for no: "))
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

robot, client_id = setup()

print("Setup Complete")
startTime = time.time()
xData = list()
yData = list()
while time.time() - startTime < 5:
    robot.motors[Joints.Right_Thigh_Kick_Joint.value].move(90)
    robot.motors[Joints.Right_Knee_Joint.value].move(-90)
    robot.motors[Joints.Right_Ankle_Joint.value].move(0)
    xData.append(time.time() - startTime)
    yData.append(-math.pi/2 - robot.motors[Joints.Right_Knee_Joint.value].get_position())
    #print(robot.motors[Joints.Right_Knee_Joint.value].get_position())

plt.scatter(xData, yData)
plt.show()
"""
Old Code, using for potential reference. Delete when robust traj planner is created

res = vrep.simx_return_novalue_flag
while res != vrep.simx_return_ok:
    res, data = vrep.simxGetObjectPosition(client_id, robot.motors[15].handle, robot.motors[8].handle, vrep.simx_opmode_streaming)

#home: 0  -10  10
moveRightLeg(robot, client_id, -90, -90, -90, 4)"""