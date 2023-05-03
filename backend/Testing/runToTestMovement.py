import sys
import time

from backend.KoalbyHumanoid.Robot import RealRobot
from backend.Primitives.MovementManager import play_motion
from backend.Simulation import sim as vrep
from backend.KoalbyHumanoid.Robot import SimRobot


def setup():
    pose_time = float(input("Enter pose time in seconds: "))
    pose_delay = float(input("Enter delay between pose time in seconds: "))
    file_name = str(input("Input saved file name to play back: "))
    simulation_flag = int(input("Are you running the Simulation?? Please enter 1 for yes and 0 for no: "))
    if simulation_flag == 1:
        vrep.simxFinish(-1)  # just in case, close all opened connections
        client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        print(client_id)  # if 1, then we are connected.

        if client_id != -1:
            print("Connected to remote API server")
        else:
            sys.exit("Not connected to remote API server")

        robot = SimRobot(client_id)  # inits sim robot
        handle = vrep.simxGetObjectHandle(client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]  # gets ID of sim cart
        vrep.simxSetObjectFloatParameter(client_id, handle, vrep.sim_shapefloatparam_mass, 5,
                                         vrep.simx_opmode_blocking)  # sets mass of sim cart to 5
    else:  # inits real-world robot
        robot = RealRobot()
        client_id = -1

    # if simulation_flag == 1:
    #     vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot)
    return robot, file_name, pose_time, pose_delay, client_id, simulation_flag


# TODO: tweaks for some reason when executing movement

robot_type, playback_file, p_time, p_delay, _, _ = setup()
time.sleep(3)
play_motion(robot_type, playback_file, p_time, p_delay)
print("Done")
