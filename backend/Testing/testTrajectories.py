import sys, time, math 
sys.path.append("./")
from backend.Testing import initSim, initRobot
from backend.KoalbyHumanoid.Robot import Joints
from backend.LimbTrajectories.rightLegTraj import * # Old Traj Code, see bottom comment
import matplotlib.pyplot as plt

# Edit to declare if you are testing the sim or the real robot
isSim = True

robot, client_id = initSim.setup() if isSim else initRobot.setup()

print("Setup Complete")
startTime = time.time()
xData = list()
yData = list()
while time.time() - startTime < 5:
    # robot.motors[Joints.Right_Thigh_Kick_Joint.value].45move(90)
    # robot.motors[Joints.Right_Knee_Joint.value].move(-90)
    # robot.motors[Joints.Right_Ankle_Joint.value].move(0)
    motorsTarget = 0
    robot.moveAllTo(motorsTarget)
    xData.append(time.time() - startTime)
    yData.append(motorsTarget - robot.motors[Joints.Lower_Torso_Side2Side_Joint.value].get_position())
    #print(robot.motors[Joints.Right_Knee_Joint.value].get_position())

plt.plot(xData, yData)
#plt.plot(xData, yData*0)
plt.title("Lower Torso - P: 10, I: 0, D: 50")
plt.xlabel("Time (seconds)")
plt.ylabel("Position Error (radians)")
plt.grid()
plt.show()




"""
Old Code, using for potential reference. Delete when robust traj planner is created

res = vrep.simx_return_novalue_flag
while res != vrep.simx_return_ok:
    res, data = vrep.simxGetObjectPosition(client_id, robot.motors[15].handle, robot.motors[8].handle, vrep.simx_opmode_streaming)

#home: 0  -10  10
moveRightLeg(robot, client_id, -90, -90, -90, 4)"""