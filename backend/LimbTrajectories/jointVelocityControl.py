import sys
import time
import math
sys.path.append("./")
from backend.Simulation import sim as vrep
from backend.KoalbyHumanoid.Kinematics.TrajectoryPlanning import TrajPlanner


class Joint:
    def __init__(self, motor, kp, ki, kd, client_id):
        self.client_id = client_id
        self.motor = motor
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.effort = 0
        self.prevError = 0
        self.prevTime = 0
        self.errorSum = 0
        self.target = 0
        self.currentPosition = 0
    
    def startJointStreaming(robot, client_id):
        for motor in robot.motors:
            print("Beginning to stream ", motor.motor_id)
            if motor.motor_id == 19: ## Motor does not exist in CoppeliaSim but does exist in Config.py. I am hesitant to delete it, so this is a bandaid fix. -Scott
                continue
            res = vrep.simx_return_novalue_flag
            while res != vrep.simx_return_ok:
                res, data = vrep.simxGetJointPosition(client_id, motor.handle, vrep.simx_opmode_streaming)

    def move(self, target):
        self.target = math.radians(target)
        actual = vrep.simxGetJointPosition(self.client_id, self.motor.handle, vrep.simx_opmode_buffer)[1]
        error = self.target - actual
        p = error * self.kp
        
        elapsedTime = time.perf_counter() - self.prevTime
        dedt = (self.prevError - error) / (self.prevTime - elapsedTime)
        d = self.kd * dedt
        
        self.effort = p + d
        if(self.effort > 4):
            self.effort = 4
        elif(self.effort < -4):
            self.effort = -4
        vrep.simxSetJointTargetVelocity(self.client_id, self.motor.handle, self.effort, vrep.simx_opmode_streaming)
        self.prevError = error
        self.prevTime = time.perf_counter()
    
    def moveWithTrajectory(self, coefficients, elapsedTime):
        angle = coefficients[0] + (coefficients[1] * elapsedTime) + (coefficients[2] * (elapsedTime ** 2)) + (coefficients[3] * (elapsedTime ** 3))
        self.move(angle)
        pass


    def moveGyro(self, target, axis):
        _, gyroSensorHandle = vrep.simxGetObjectHandle(self.client_id, 'GyroSensor', vrep.simx_opmode_blocking)
        if gyroSensorHandle == -1:
            print("failed to get gyroscope sensor handle")
        res = vrep.simx_return_novalue_flag
        if axis == 'x':
            gyroData = vrep.simxGetFloatSignal(self.client_id, 'gyroX', vrep.simx_opmode_buffer)[1]
        elif axis == 'y':
            gyroData = vrep.simxGetFloatSignal(self.client_id, 'gyroY', vrep.simx_opmode_buffer)[1]
            
        error = target - gyroData
        p = error * self.kp
        
        self.errorSum += error
        if self.errorSum > 20:
            self.errorSum = 20
        i = self.errorSum * self.ki
        
        effort = p + i
        max = 2
        if(effort > max):
            effort = max
        elif(effort < -max):
            effort = -max
        vrep.simxSetJointTargetVelocity(self.client_id, self.motor.handle, effort, vrep.simx_opmode_streaming)
        
    def getGyroValues(client_id):
        X = vrep.simxGetFloatSignal(client_id, 'gyroX', vrep.simx_opmode_buffer)[1]
        Y = vrep.simxGetFloatSignal(client_id, 'gyroY', vrep.simx_opmode_buffer)[1]
        Z = vrep.simxGetFloatSignal(client_id, 'gyroZ', vrep.simx_opmode_buffer)[1]
        return [X, Y, Z]

    def setPosition(self, angle):
        vrep.simxSetJointTargetPosition(self.client_id, self.motor.handle, angle, vrep.simx_opmode_streaming)
    
    
        
