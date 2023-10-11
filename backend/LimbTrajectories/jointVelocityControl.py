import sys
import time
import math
sys.path.insert(0, "C:\\Users\\scott\\Desktop\\flask-project-main\\flask-project-main")
from backend.Simulation import sim as vrep
from backend.KoalbyHumanoid.Kinematics.TrajectoryPlanning import TrajPlanner


class jointPIDControl:
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
    
    def move(self, target):
        target = math.radians(target)
        res = vrep.simx_return_novalue_flag
        while res != vrep.simx_return_ok:
            res, actual = vrep.simxGetJointPosition(self.client_id, self.motor.handle, vrep.simx_opmode_streaming)
        error = target - actual
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
        
    def updateGyro(client_id):
        X = vrep.simxGetFloatSignal(client_id, 'gyroX', vrep.simx_opmode_buffer)[1]
        Y = vrep.simxGetFloatSignal(client_id, 'gyroY', vrep.simx_opmode_buffer)[1]
        Z = vrep.simxGetFloatSignal(client_id, 'gyroZ', vrep.simx_opmode_buffer)[1]
        return [X, Y, Z]

    def setPosition(self, angle):
        vrep.simxSetJointTargetPosition(self.client_id, self.motor.handle, angle, vrep.simx_opmode_streaming)
    
    
        
