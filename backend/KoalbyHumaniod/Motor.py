"""The Motor class hold all information for an abstract motor on the physical robot. It is used to interface with the
arduino which directly controls the motors"""
from abc import ABC, abstractmethod

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
    def __init__(self, motor_id, handle):
        self.handle = handle
        # super().__init__(self, motor_id) # idk why this doesn't work/how to make it work
        self.motor_id = motor_id

    def get_position(self, client_id):
        """reads the motor's current position from the Simulation and returns the value in degrees"""
        return vrep.simxGetJointPosition(client_id, self.handle, vrep.simx_opmode_streaming)

    def set_position(self, position, client_id):
        """sends a desired motor position to the Simulation"""
        # idk why you have to divide the motor position by a constant but it freaks out if not
        vrep.simxSetJointTargetPosition(client_id, self.handle, position / 40, vrep.simx_opmode_streaming)
        # pose_time not used -- could do something with velocity but unsure if its necessary to go through


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
