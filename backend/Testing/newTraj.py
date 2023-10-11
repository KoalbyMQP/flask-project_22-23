import sys
sys.path.insert(0, "C:\\Users\\sbpen\\OneDrive\\Desktop\\flask-project-main\\flask-project-main\\flask-project-main")
#sys.path.insert(0, r"./")
from backend.KoalbyHumanoid.Robot import RealRobot, SimRobot
from backend.Simulation import sim as vrep
from backend.LimbTrajectories.rightLegTraj import *
from backend.LimbTrajectories.jointVelocityControl import *

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
        handle = vrep.simxGetObjectHandle(client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]  # gets ID of sim cart
        vrep.simxSetObjectFloatParameter(client_id, handle, vrep.sim_shapefloatparam_mass, 5,
                                         vrep.simx_opmode_blocking)  # sets mass of sim cart to 5
        

    else:  # inits real-world robot
        robot = RealRobot()
        client_id = -1

    # if simulation_flag == 1:
    #     vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot)
    return robot, client_id


robot, client_id = setup()
print("setup complete")

rightKick = jointPIDControl(robot.motors[15], 1, 0, 10, client_id)
while True:
    rightKick.move(45)

"""res = vrep.simx_return_novalue_flag
while res != vrep.simx_return_ok:
    res, data = vrep.simxGetObjectPosition(client_id, robot.motors[15].handle, robot.motors[8].handle, vrep.simx_opmode_streaming)

#home: 0  -10  10
moveRightLeg(robot, client_id, -90, -90, -90, 4)"""