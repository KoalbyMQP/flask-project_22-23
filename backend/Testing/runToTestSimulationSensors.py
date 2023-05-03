import math
import sys
import time

from backend.KoalbyHumanoid.Sensors.PiratedCode.BoardDisplay_EKF import initializeCube, ProjectionViewer
from backend.Simulation import sim as vrep
from backend.KoalbyHumanoid.Robot import SimRobot


def run_sim_sensors():
    '''
    Initializes and runs the sensors in simulation
    '''
    vrep.simxFinish(-1)  # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # inits simulation
    if client_id != -1:  # TODO: Fix duplicate code
        print("Connected to remote API server")
    else:
        sys.exit("Not connected to remote API server")

    robot = SimRobot(client_id)  # inits sim robot
    handle = vrep.simxGetObjectHandle(client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]  # gets ID of sim cart
    vrep.simxSetObjectFloatParameter(client_id, handle, vrep.sim_shapefloatparam_mass, 5, vrep.simx_opmode_blocking)  # gives sim cart a mass of 5
    # vrep.simxSetObjectParent('Right_Forearm', handle, False)
    # vrep.simxSetObjectParent('Left_Forearm', handle, False)

    # filtered
    block = initializeCube()  # UNSURE WHAT THIS DOES SOMEONE COMMENT THIS
    pv = ProjectionViewer(640, 480, block)
    print("This will go on forever. Simulation and code needs to be manually stopped")
    pv.run(robot, client_id, 1)

    # not filtered
    # while True:
    #     data = robot.get_imu_data()
    #     # pitch_rad = math.asin(data[3]/(math.sqrt((data[3]*data[3]) + (data[4] * data[4]) + (data[5] * data[5]))))
    #     pitch_rad = math.asin(data[3] / 9.81)
    #     pitch = math.degrees(pitch_rad)
    #     roll_rad = math.atan(data[4]/data[5])
    #     roll = math.degrees(roll_rad)
    #     print(str(pitch) + "," + str(roll))
    #     time.sleep(.5)



run_sim_sensors()
