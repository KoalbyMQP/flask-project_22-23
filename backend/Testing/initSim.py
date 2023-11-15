import sys
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import SimRobot
from backend.Simulation import sim as vrep
#from backend.LimbTrajectories.jointVelocityControl import Joint

"""
Function to connect Python to CoppeliaSim. Does not run if user is trying to connect to the real robot
"""
def setup():
    vrep.simxFinish(-1)  # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1', 19997, True, True, -500000, 5)
    # client_id = vrep.simxStartSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    if client_id != -1:
        print("Connected to remote API server")
        #vrep.simxStartSimulation(client_id, operationMode=vrep.simx_opmode_oneshot)
        #vrep.simxPauseSimulation(client_id, operationMode=vrep.simx_opmode_oneshot)
    else:
        sys.exit("Not connected to remote API server")
    robot = SimRobot(client_id)  # inits sim robot
    return robot, client_id