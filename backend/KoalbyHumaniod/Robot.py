import time
from abc import ABC, abstractmethod

import backend.KoalbyHumaniod.Config as config
from backend.ArduinoSerial import ArduinoSerial
from backend.KoalbyHumaniod.Motor import RealMotor, SimMotor
from backend.Simulation import sim as vrep
from backend.KoalbyHumaniod.Sensors.PiratedCode import Kalman_EKF as km


class Robot(ABC):
    def __init__(self):
        print("Robot Created and Initialized")
        pass

    @abstractmethod
    def update_motors(self, pose_time_millis, motor_positions_dict):
        pass

    @abstractmethod
    def motors_init(self):
        pass

    @abstractmethod
    def shutdown(self):
        pass

    @abstractmethod
    def get_imu_data(self):
        pass

    @abstractmethod
    def read_battery_level(self):
        pass

    @abstractmethod
    def get_tf_luna_data(self):
        pass

    @abstractmethod
    def get_husky_lens_data(self):
        pass

    @abstractmethod
    def get_filtered_data(self, data):
        pass

    @abstractmethod
    def open_hand(self):
        pass

    @abstractmethod
    def close_hand(self):
        pass

    @abstractmethod
    def stop_hand(self):
        pass


class SimRobot(Robot):
    def __init__(self, client_id):
        super().__init__()
        self.client_id = client_id
        self.primitives = []
        self.is_real = False
        self.motors = self.motors_init()
        self.sys = km.System()
        # print(client_id)

    def motors_init(self):
        motors = list()
        for motorConfig in config.motors:
            # handle = vrep.simxGetObjectHandle(self.client_id, motorConfig[3], vrep.simx_opmode_blocking)[1]
            # print(self.client_id)
            vrep.simxSetObjectFloatParameter(self.client_id, vrep.simxGetObjectHandle(self.client_id, motorConfig[3],
                                                                                      vrep.simx_opmode_blocking)[1],
                                             vrep.sim_shapefloatparam_mass, 1,
                                             vrep.simx_opmode_blocking)
            motor = SimMotor(motorConfig[0],
                             vrep.simxGetObjectHandle(self.client_id, motorConfig[3], vrep.simx_opmode_blocking)[1])
            setattr(SimRobot, motorConfig[3], motor)
            motors.append(motor)
        return motors

    def update_motors(self, pose_time_millis, motor_positions_dict):
        """
        Take the primitiveMotorDict and send the motor values to the robot
        """

        for key, value in motor_positions_dict.items():
            for motor in self.motors:
                if str(motor.motor_id) == str(key):
                    motor.set_position(value, self.client_id)

    def shutdown(self):
        vrep.simxStopSimulation(self.client_id, vrep.simx_opmode_oneshot)

    def get_imu_data(self):
        data = [vrep.simxGetFloatSignal(self.client_id, "gyroX", vrep.simx_opmode_streaming)[1] + .001,
                vrep.simxGetFloatSignal(self.client_id, "gyroY", vrep.simx_opmode_streaming)[1] + .001,
                vrep.simxGetFloatSignal(self.client_id, "gyroZ", vrep.simx_opmode_streaming)[1] + .001,
                vrep.simxGetFloatSignal(self.client_id, "accelerometerX", vrep.simx_opmode_streaming)[1] + .001,
                vrep.simxGetFloatSignal(self.client_id, "accelerometerY", vrep.simx_opmode_streaming)[1] + .001,
                vrep.simxGetFloatSignal(self.client_id, "accelerometerZ", vrep.simx_opmode_streaming)[1] + .001, 1, 1,
                1]
        # have to append 1 for magnetometer data because there isn't one in CoppeliaSim
        return data

    def read_battery_level(self):
        pass

    def get_tf_luna_data(self):
        dist = float(vrep.simxGetFloatSignal(self.client_id, "proximity", vrep.simx_opmode_streaming)[1])
        print(dist)
        if dist > 5:
            print("stop")

    def get_husky_lens_data(self):
        pass

    def get_filtered_data(self, data):

        w = [data[0], data[1], data[2]]  # gyro
        dt = 1 / 50
        a = [data[3], data[4], data[5]]  # accele
        m = [data[6], data[7], data[8]]  # magnetometer
        # quat rotate
        self.sys.predict(w, dt)  # w = gyroscope
        self.sys.update(a, m)  # a = acceleration, m = magnetometer
        return km.getEulerAngles(self.sys.xHat[0:4])

    def open_hand(self):
        pass

    def close_hand(self):
        pass

    def stop_hand(self):
        pass


class RealRobot(Robot):

    def __init__(self):
        super().__init__()
        print("here")
        self.primitives = []
        self.is_real = True
        self.arduino_serial = ArduinoSerial()
        self.motors = self.motors_init()
        self.sys = km.System()
        self.arduino_serial.send_command('1,')  # This initializes the robot with all the initial motor positions
        self.arduino_serial.send_command('40')  # Init IMU
        time.sleep(2)
        self.arduino_serial.send_command('50')  # Init TFLuna
        time.sleep(2)
        print(self.arduino_serial.read_command())
        print(self.arduino_serial.read_command())
        self.arduino_serial.send_command('60')  # Init HuskyLens
        print(self.arduino_serial.read_command())
        print("Huskey Lens Init")
        self.left_hand_motor = None

    def motors_init(self):

        motors = list()
        for motorConfig in config.motors:
            #               motorID        angleLimit         name              serial
            motor = RealMotor(motorConfig[0], motorConfig[1], motorConfig[3], self.arduino_serial)
            setattr(RealRobot, motorConfig[3], motor)
            motors.append(motor)
            if motorConfig[3] == "Left_Hand_Joint":
                self.left_hand_motor = motor
        print("Motors initialized")
        # print(motors)
        return motors

    def update_motors(self, pose_time_millis, motor_positions_dict):
        """
        Take the primitiveMotorDict and send the motor values to the robot
        """
        # very similar to sim update -- could abstract if needed
        for key, value in motor_positions_dict.items():
            for motor in self.motors:
                if str(motor.motor_id) == str(key):
                    #                               position                  time
                    # every position in here is one less
                    motor.set_position_time(motor_positions_dict[key], pose_time_millis)

    def shutdown(self):
        self.arduino_serial.send_command('100')

    def get_imu_data(self):
        data = []
        self.arduino_serial.send_command('41')  # reads IMU data
        string_data = self.arduino_serial.read_command()
        if string_data.__len__() == 0:
            return
        num_data = string_data.split(",")
        for piece in num_data:
            num_piece = float(piece)
            if num_piece != 0:
                data.append(num_piece)
            else:
                data.append(.01)
        return data

    def read_battery_level(self):
        self.arduino_serial.send_command("30")
        return self.arduino_serial.read_command()

    def get_tf_luna_data(self):

        self.arduino_serial.send_command('51')  # reads TFLuna data
        time.sleep(1)
        return self.arduino_serial.read_command()

        # print(string_data)
        # if string_data.__len__() == 0:
        #     return
        # num_data = string_data.split(",")
        # for piece in num_data:
        #     check += float(piece)
        # if data[8] == (check & 0xff):
        #     dist = data[2] + data[3] * 256
        # return dist

    def get_husky_lens_data(self):
        self.arduino_serial.send_command("61")
        return self.arduino_serial.read_command()

    def get_filtered_data(self, data):
        w = [data[0], data[1], data[2]]  # gyro
        dt = 1 / 50
        a = [data[3], data[4], data[5]]  # accele
        m = [data[6], data[7], data[8]]  # magnetometer
        # quat rotate
        self.sys.predict(w, dt)  # w = gyroscope
        self.sys.update(a, m)  # a = acceleration, m = magnetometer
        return km.getEulerAngles(self.sys.xHat[0:4])

    def open_hand(self):
        self.left_hand_motor.rotation_on(10)

    def close_hand(self):
        self.left_hand_motor.rotation_on(-10)

    def stop_hand(self):
        self.left_hand_motor.rotation_off()
