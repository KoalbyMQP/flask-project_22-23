import sys, time, math 
sys.path.append("./")
from backend.Testing import initSim, initRobot, trajPlanner
from backend.Testing import inverseKinematics as IK
from backend.KoalbyHumanoid.Robot import Joints
from backend.LimbTrajectories.rightLegTraj import * # Old Traj Code, see bottom comment
import matplotlib.pyplot as plt


# Edit to declare if you are testing the sim or the real robot
isSim = True

robot, client_id = initSim.setup() if isSim else initRobot.setup()

print("Setup Complete")

setPoints = [[0,  0], [math.radians(-90), math.radians(-90)], [0,  0]]
tj = trajPlanner.TrajPlannerNew(setPoints)
traj = tj.getCubicTraj(10, 100)

robot.motors[1].target = math.radians(80)
robot.motors[6].target = math.radians(-80)
# robot.motors[3].target = math.radians(90)
# robot.motors[8].target = math.radians(90)
robot.motors[14].target = 0

#robot.motors[17].target = math.radians(-45)
#robot.motors[22].target = math.radians(45)
prevTime = time.time()
vrep.simxStartSimulation(client_id, operationMode=vrep.simx_opmode_oneshot)

print(robot.locate(robot.motors[3]))


#robot.motors[0].target = -math.radians(90)
simStartTime = time.time()
while time.time() - simStartTime < 5:
    time.sleep(0.01)
    robot.updateRobotCoM() 
    robot.IMUBalance()
    robot.moveAllToTarget()

while True:
    errorData = []
    timeData = []
    startTime = time.time()
    for point in traj:
        time.sleep(0.01)
        robot.updateRobotCoM()
        errorData.append(robot.balanceAngle())
        timeData.append(time.time() - simStartTime)
        #print(robot.motors[0].prevError, robot.motors[0].effort)
        robot.moveAllToTarget()
        #print(robot.CoM)

        # print(point)
        robot.motors[0].target = point[1]
        # robot.motors[3].target += math.radians(2)
        robot.motors[5].target = point[2]
        # robot.motors[8].target += math.radians(2)
        #print(math.degrees(robot.motors[0].target))
        while time.time() - startTime < point[0]:
            time.sleep(0.01)
            robot.updateRobotCoM() 
            errorData.append(robot.balanceAngle())
            timeData.append(time.time() - simStartTime)
            #print(robot.motors[0].prevError, robot.motors[0].effort)
            robot.moveAllToTarget()
            #print(robot.locate(robot.motors[22]))
            #print(robot.CoM)
print("done")

plt.plot(timeData,errorData)
plt.show()
exit(0)



startTime = time.time()
for point in traj:
    motorsTarget = point[1]
    #print(motorsTarget)
    tarTime = point[0]
    curTime = time.time()
    while curTime - startTime <= tarTime:
        robot.motors[Joints.Right_Shoulder_Rotator_Joint.value].move(motorsTarget)
        curTime = time.time()
        # print(curTime - startTime)

exit(0)

xData = list()
yData = list()
startTime = time.time()
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