from threading import Thread
import sys

from backend.KoalbyHumanoid.Robot import RealRobot
from backend.simulation import sim as vrep
from backend.KoalbyHumanoid.Robot import SimRobot
from backend.Primitives.MovementManager import play_motion
from backend.testing.runToTestKinematics import Walker
from backend.testing.runToTestSensors import pv
from backend.KoalbyHumanoid.Kinematics.TrajectoryPlanning import TrajPlanner
from backend.Primitives.MovementManager import play_motion_kinematics

# initialize Walker
walker = Walker(True)
walker.init_sim()


poseTime = float(input("Enter pose time in seconds: "))  # TODO: fix duplicated code
poseDelay = float(input("Enter delay between pose time in seconds: "))
fileName = str(input("Input saved file name to play back: "))
simulationFlag = int(input("Are you running the simulation?? Please enter 1 for yes and 0 for no: "))
if simulationFlag == 1:  # inits sim robot
    vrep.simxFinish(-1)  # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    print(client_id)  # if 1, then we are connected.

    if client_id != -1:
         print("Connected to remote API server")
    else:
        sys.exit("Not connected to remote API server")

    robot = SimRobot(client_id)  # inits sim robot
    handle = vrep.simxGetObjectHandle(client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]  # gets sim cart ID
    vrep.simxSetObjectFloatParameter(client_id, handle, vrep.sim_shapefloatparam_mass, 5, vrep.simx_opmode_blocking)  # gives sim cart a mass of 5
else:  # inits real-world robot
    robot = RealRobot()

play_motion(robot, fileName, poseTime, poseDelay)  # plays a recorded motion from a CSV file
thready = Thread(target=pv.run(robot, simulationFlag, client_id))  # create thread for each function
thready.start()  # run thread # TODO: need to figure out how to actaully thread this. Doesn't start until visualizer board is stopped
