import sys
import time
import math
sys.path.insert(0, "C:\\Users\\scott\\Desktop\\flask-project-main\\flask-project-main")
from backend.Simulation import sim as vrep
from backend.KoalbyHumanoid.Kinematics.TrajectoryPlanning import TrajPlanner

def moveBothLegs(robot, client_id, right_desired, left_desired, tf):
    # Creates variables to call proper motors
    rightKick = robot.motors[15]
    rightKnee = robot.motors[16]
    rightAnkle = robot.motors[17]
    
    leftKick = robot.motors[20]
    leftKnee = robot.motors[21]
    leftAnkle = robot.motors[22]
    
    left_kickPose = vrep.simxGetJointPosition(client_id, leftKick.handle, vrep.simx_opmode_streaming)[1]
    left_kneePose = vrep.simxGetJointPosition(client_id, leftKnee.handle, vrep.simx_opmode_streaming)[1]
    left_anklePose = vrep.simxGetJointPosition(client_id, leftAnkle.handle, vrep.simx_opmode_streaming)[1]
    right_kickPose = vrep.simxGetJointPosition(client_id, rightKick.handle, vrep.simx_opmode_streaming)[1]
    right_kneePose = vrep.simxGetJointPosition(client_id, rightKnee.handle, vrep.simx_opmode_streaming)[1]
    right_anklePose = vrep.simxGetJointPosition(client_id, rightAnkle.handle, vrep.simx_opmode_streaming)[1]
    res = vrep.simx_return_novalue_flag
    while res != vrep.simx_return_ok:
        res, data = vrep.simxGetJointPosition(client_id, rightAnkle.handle, vrep.simx_opmode_streaming)


    # HOME = 10, -20, 10


    ##Gets starting joint values IN RADIANS
    right_kickPose = vrep.simxGetJointPosition(client_id, rightKick.handle, vrep.simx_opmode_streaming)[1]
    right_kneePose = vrep.simxGetJointPosition(client_id, rightKnee.handle, vrep.simx_opmode_streaming)[1]
    right_anklePose = vrep.simxGetJointPosition(client_id, rightAnkle.handle, vrep.simx_opmode_streaming)[1]
    left_kickPose = vrep.simxGetJointPosition(client_id, leftKick.handle, vrep.simx_opmode_streaming)[1]
    left_kneePose = vrep.simxGetJointPosition(client_id, leftKnee.handle, vrep.simx_opmode_streaming)[1]
    left_anklePose = vrep.simxGetJointPosition(client_id, leftAnkle.handle, vrep.simx_opmode_streaming)[1]

    right_kickCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(right_kickPose), right_desired[0], 0, 0)
    right_kneeCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(right_kneePose), right_desired[1], 0, 0)
    right_ankleCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(right_anklePose), right_desired[2], 0, 0)
    left_kickCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(left_kickPose), left_desired[0], 0, 0)
    left_kneeCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(left_kneePose), left_desired[1], 0, 0)
    left_ankleCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(left_anklePose), left_desired[2], 0, 0)


    time.sleep(1)
    prevTime = time.perf_counter()
    elapsedTime = time.perf_counter() - prevTime
    while elapsedTime < tf:
        
        #Degrees
        kickAngle = right_kickCoeffs[0] + (right_kickCoeffs[1] * elapsedTime) + (right_kickCoeffs[2] * (elapsedTime ** 2)) + (right_kickCoeffs[3] * (elapsedTime ** 3))
        kneeAngle = right_kneeCoeffs[0] + (right_kneeCoeffs[1] * elapsedTime) + (right_kneeCoeffs[2] * (elapsedTime ** 2)) + (right_kneeCoeffs[3] * (elapsedTime ** 3))
        ankleAngle = right_ankleCoeffs[0] + (right_ankleCoeffs[1] * elapsedTime) + (right_ankleCoeffs[2] * (elapsedTime ** 2)) + (right_ankleCoeffs[3] * (elapsedTime ** 3))

        rightKick.set_position(math.radians(kickAngle), client_id)
        rightKnee.set_position(math.radians(kneeAngle), client_id)
        rightAnkle.set_position(math.radians(ankleAngle), client_id)
        
        #Degrees
        kickAngle = left_kickCoeffs[0] + (left_kickCoeffs[1] * elapsedTime) + (left_kickCoeffs[2] * (elapsedTime ** 2)) + (left_kickCoeffs[3] * (elapsedTime ** 3))
        kneeAngle = left_kneeCoeffs[0] + (left_kneeCoeffs[1] * elapsedTime) + (left_kneeCoeffs[2] * (elapsedTime ** 2)) + (left_kneeCoeffs[3] * (elapsedTime ** 3))
        ankleAngle = left_ankleCoeffs[0] + (left_ankleCoeffs[1] * elapsedTime) + (left_ankleCoeffs[2] * (elapsedTime ** 2)) + (left_ankleCoeffs[3] * (elapsedTime ** 3))

        leftKick.set_position(math.radians(kickAngle), client_id)
        leftKnee.set_position(math.radians(kneeAngle), client_id)
        leftAnkle.set_position(math.radians(ankleAngle), client_id)
        
        
        time.sleep(0.05)
        elapsedTime = time.perf_counter() - prevTime
        
    print("Done!")
    

def moveRightLeg(robot, client_id, desiredKick, desiredKnee, desiredAnkle, tf):
    # Creates variables to call proper motors
    rightKick = robot.motors[15]
    rightKnee = robot.motors[16]
    rightAnkle = robot.motors[17]

    # HOME = 10, -20, 10
        
    kickPose = vrep.simxGetJointPosition(client_id, rightKick.handle, vrep.simx_opmode_streaming)[1]
    kneePose = vrep.simxGetJointPosition(client_id, rightKnee.handle, vrep.simx_opmode_streaming)[1]
    anklePose = vrep.simxGetJointPosition(client_id, rightAnkle.handle, vrep.simx_opmode_streaming)[1]
    res = vrep.simx_return_novalue_flag
    while res != vrep.simx_return_ok:
        res, data = vrep.simxGetJointPosition(client_id, rightAnkle.handle, vrep.simx_opmode_streaming)

    ##Gets starting joint values IN RADIANS
    kickPose = vrep.simxGetJointPosition(client_id, rightKick.handle, vrep.simx_opmode_streaming)[1]
    kneePose = vrep.simxGetJointPosition(client_id, rightKnee.handle, vrep.simx_opmode_streaming)[1]
    anklePose = vrep.simxGetJointPosition(client_id, rightKnee.handle, vrep.simx_opmode_streaming)[1]

    kickCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(kickPose), desiredKick, 0, 0)
    kneeCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(kneePose), desiredKnee, 0, 0)
    ankleCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(anklePose), desiredAnkle, 0, 0)

    time.sleep(1)
    prevTime = time.perf_counter()
    elapsedTime = time.perf_counter() - prevTime
    while elapsedTime < tf:
        
        #Degrees
        kickAngle = kickCoeffs[0] + (kickCoeffs[1] * elapsedTime) + (kickCoeffs[2] * (elapsedTime ** 2)) + (kickCoeffs[3] * (elapsedTime ** 3))
        kneeAngle = kneeCoeffs[0] + (kneeCoeffs[1] * elapsedTime) + (kneeCoeffs[2] * (elapsedTime ** 2)) + (kneeCoeffs[3] * (elapsedTime ** 3))
        ankleAngle = ankleCoeffs[0] + (ankleCoeffs[1] * elapsedTime) + (ankleCoeffs[2] * (elapsedTime ** 2)) + (ankleCoeffs[3] * (elapsedTime ** 3))

        rightKick.set_position(math.radians(kickAngle), client_id)
        rightKnee.set_position(math.radians(kneeAngle), client_id)
        rightAnkle.set_position(math.radians(ankleAngle), client_id)
        
        time.sleep(0.05)
        elapsedTime = time.perf_counter() - prevTime

    print("Done!")


def moveLeftLeg(robot, client_id, desiredKick, desiredKnee, desiredAnkle, tf):
    # Creates variables to call proper motors
    leftKick = robot.motors[20]
    leftKnee = robot.motors[21]
    leftAnkle = robot.motors[22]

    # HOME = 10, -20, 10
        
    kickPose = vrep.simxGetJointPosition(client_id, leftKick.handle, vrep.simx_opmode_streaming)[1]
    kneePose = vrep.simxGetJointPosition(client_id, leftKnee.handle, vrep.simx_opmode_streaming)[1]
    anklePose = vrep.simxGetJointPosition(client_id, leftAnkle.handle, vrep.simx_opmode_streaming)[1]
    res = vrep.simx_return_novalue_flag
    while res != vrep.simx_return_ok:
        res, data = vrep.simxGetJointPosition(client_id, leftAnkle.handle, vrep.simx_opmode_streaming)

    ##Gets starting joint values IN RADIANS
    kickPose = vrep.simxGetJointPosition(client_id, leftKick.handle, vrep.simx_opmode_streaming)[1]
    kneePose = vrep.simxGetJointPosition(client_id, leftKnee.handle, vrep.simx_opmode_streaming)[1]
    anklePose = vrep.simxGetJointPosition(client_id, leftAnkle.handle, vrep.simx_opmode_streaming)[1]

    kickCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(kickPose), desiredKick, 0, 0)
    kneeCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(kneePose), desiredKnee, 0, 0)
    ankleCoeffs = TrajPlanner.generate_cubic_traj_coefficients(0, tf, math.degrees(anklePose), desiredAnkle, 0, 0)

    time.sleep(1)
    prevTime = time.perf_counter()
    elapsedTime = time.perf_counter() - prevTime
    while elapsedTime < tf:
        
        #Degrees
        kickAngle = kickCoeffs[0] + (kickCoeffs[1] * elapsedTime) + (kickCoeffs[2] * (elapsedTime ** 2)) + (kickCoeffs[3] * (elapsedTime ** 3))
        kneeAngle = kneeCoeffs[0] + (kneeCoeffs[1] * elapsedTime) + (kneeCoeffs[2] * (elapsedTime ** 2)) + (kneeCoeffs[3] * (elapsedTime ** 3))
        ankleAngle = ankleCoeffs[0] + (ankleCoeffs[1] * elapsedTime) + (ankleCoeffs[2] * (elapsedTime ** 2)) + (ankleCoeffs[3] * (elapsedTime ** 3))

        leftKick.set_position(math.radians(kickAngle), client_id)
        leftKnee.set_position(math.radians(kneeAngle), client_id)
        leftAnkle.set_position(math.radians(ankleAngle), client_id)
        
        time.sleep(0.05)
        elapsedTime = time.perf_counter() - prevTime

    print("Done!")


