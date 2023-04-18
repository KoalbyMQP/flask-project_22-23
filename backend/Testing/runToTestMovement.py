import sys
from backend.KoalbyHumaniod.Robot import RealRobot
from backend.Primitives.MovementManager import play_motion
from backend.simulation import sim as vrep
from backend.KoalbyHumaniod.Robot import SimRobot


def execute_movement():
    """
    Plays a motion using positions from a user-input CSV file
    """
    poseTime = float(input("Enter pose time in seconds: "))
    poseDelay = float(input("Enter delay between pose time in seconds: "))
    fileName = str(input("Input saved file name to play back: "))
    simulationFlag = int(input("Are you running the simulation?? Please enter 1 for yes and 0 for no: "))
    if simulationFlag == 1:  # inits simulation
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

    play_motion(robot, fileName, poseTime, poseDelay)  # plays a motion from a CSV file on the robot

    # if simulationFlag == 1:
    #     vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot)


# TODO: tweaks for some reason when executing movement

execute_movement()
print("Done")
