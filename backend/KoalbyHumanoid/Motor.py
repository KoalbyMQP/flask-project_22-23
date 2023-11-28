"""The Motor class hold all information for an abstract motor on the physical robot. It is used to interface with the
arduino which directly controls the motors"""
from abc import ABC, abstractmethod
import math
import time
import numpy as np
from backend.Simulation import sim as vrep


class Motor(ABC):
    def __int__(self, motor_id):
        self.motor_id = motor_id

    @abstractmethod
    def get_position(self, client_id):
        pass

    @abstractmethod
    def set_position(self, position, client_id):
        pass


class SimMotor(Motor):
    def __init__(self, motor_id, client_id, handle, pidGains, twist, mass, home):
        self.handle = handle
        # super().__init__(self, motor_id) # idk why this doesn't work/how to make it work
        self.motor_id = motor_id
        self.pidGains = pidGains
        self.client_id = client_id
        self.name = ""

        self.twist = twist
        self.mass = mass
        self.home = np.array(home)
        self.theta = None

        #Should we put this in a Controller Object? Then have the motor have a controller assigned to it?
        self.target = 0
        self.prevTime = 0
        self.prevError = 0
        self.effort = 0
        self.errorMemorySize = 100
        self.errorMemory = [0]*self.errorMemorySize
        self.errorMemoryIndex = 0

    def get_position(self):
        """reads the motor's current position from the Simulation and returns the value in Radians"""
        return vrep.simxGetJointPosition(self.client_id, self.handle, vrep.simx_opmode_buffer)[1]

    def set_position(self, position):
        """sends a desired motor position to the Simulation"""
        position = math.radians(position)
        # idk why you have to divide the motor position by a constant but it freaks out if not
        # ^^^ From 22-23 Team. Kept it as a mark of shame -23-24 team
        vrep.simxSetJointTargetPosition(self.client_id, self.handle, position, vrep.simx_opmode_streaming)
        # pose_time not used -- could do something with velocity but unsure if its necessary to go through

    def move(self, position="TARGET"):
        if time.perf_counter() - self.prevTime > 0.01:
            if position == "TARGET":
                position = self.target
            kP, kI, kD = self.pidGains
            self.theta = self.get_position()
            error = position - self.theta
            p = error * kP
            
            self.errorMemoryIndex %= self.errorMemorySize
            self.errorMemory[self.errorMemoryIndex] = error
            i = sum(self.errorMemory) * kI

            elapsedTime = time.perf_counter() - self.prevTime
            dedt = (error - self.prevError)# / (elapsedTime - self.prevTime )
            d = kD * dedt
            
            self.effort = p + i + d
            """if(self.effort > 4):
                self.effort = 4
            elif(self.effort < -4):
                self.effort = -4"""
            #print(self.effort)
            vrep.simxSetJointTargetVelocity(self.client_id, self.handle, self.effort, vrep.simx_opmode_streaming)
            self.prevError = error
            self.prevTime = time.perf_counter()
            self.errorMemoryIndex += 1

class RealMotor(Motor):
    def __init__(self, motor_id, angle_limit, name, serial):
        self.angle_limit = angle_limit
        self.name = name
        self.arduino_serial = serial
        # super().__init__(self, motor_id) # idk why this doesn't work/how to make it work
        self.motor_id = motor_id

    def get_position(self, client_id):
        """reads the motor's current position from the arduino and returns the value in degrees"""
        id_arr = [5, self.motor_id]
        self.arduino_serial.send_command(','.join(map(str, id_arr)) + ',')
        current_position = self.arduino_serial.read_command()
        return current_position

    def set_position(self, position, client_id):
        """sends a desired motor position to the arduino"""
        position = int(position)
        id_pos_arr = [10, self.motor_id, position]
        self.arduino_serial.send_command(','.join(map(str, id_pos_arr)) + ',')

    def rotation_on(self, speed):
        """sends a desired motor speed to the arduino"""
        id_pos_arr = [74, self.motor_id, speed]
        self.arduino_serial.send_command(','.join(map(str, id_pos_arr)) + ',')

    def rotation_off(self):
        """sends a desired motor speed to the arduino"""
        id_pos_arr = [76, self.motor_id, 0]
        self.arduino_serial.send_command(','.join(map(str, id_pos_arr)) + ',')

    def set_position_time(self, position, time):
        """sends a desired motor position to the arduino <to be executed in a set amount of time?>"""
        id_pos_time_arr = [11, self.motor_id, position, time]  # TODO: time only works with herkulex
        command = ','.join(map(str, id_pos_time_arr)) + ','
        self.arduino_serial.send_command(command)

    def compliant_toggle(self, toggle):
        """turns the compliance of a motor on or off based on a 1 or 0 input and sends this to the arduino"""
        id_bool_arr = [21, self.motor_id, toggle]
        self.arduino_serial.send_command(','.join(map(str, id_bool_arr)) + ',')
