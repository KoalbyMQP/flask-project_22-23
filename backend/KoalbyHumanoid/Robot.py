import time
from abc import ABC, abstractmethod

import backend.KoalbyHumanoid.Config as Config
from backend.ArduinoSerial import ArduinoSerial
from backend.KoalbyHumanoid.Motor import RealMotor, SimMotor
from backend.Simulation import sim as vrep
from backend.KoalbyHumanoid.Sensors.PiratedCode import Kalman_EKF as KM


class Robot(ABC):
    def __init__(self, is_real, motors):
        self.motors = motors
        print("Robot Created and Initialized")
        self.is_real = is_real
        self.sys = KM.System()

    def get_motor(self, key):
        for motor in self.motors:
            if motor.motor_id == key:
                return motor

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

    def get_filtered_data(self, data):
        # print("here")
        w = [data[0], data[1], data[2]]  # gyro
        dt = 1 / 50
        a = [data[3], data[4], data[5]]  # accele
        m = [data[6], data[7], data[8]]  # magnetometer
        # quat rotate

        self.sys.predict(w, dt)  # w = gyroscope
        self.sys.update(a, m)  # a = acceleration, m = magnetometer
        # return KM.getEulerAngles(self.sys.xHat[0:4])
        return KM.getEulerAngles(data)

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
        self.client_id = client_id
        self.motors = self.motors_init()
        super().__init__(False, self.motors)
        self.primitives = []
        self.is_real = False
    
        # print(client_id)

    def motors_init(self):
        motors = list()
        for motorConfig in Config.motors:
            if motorConfig[0] == 19: ## Motor does not exist in CoppeliaSim but does exist in Config.py. I am hesitant to delete it, so this is a bandaid fix. -Scott
                continue
            # handle = vrep.simxGetObjectHandle(self.client_id, motorConfig[3], vrep.simx_opmode_blocking)[1]
            # print(self.client_id)
            vrep.simxSetObjectFloatParameter(self.client_id, vrep.simxGetObjectHandle(self.client_id, motorConfig[3],
                                                                                      vrep.simx_opmode_blocking)[1],
                                             vrep.sim_shapefloatparam_mass, 1,
                                             vrep.simx_opmode_blocking)
            motor = SimMotor(motorConfig[0], self.client_id, vrep.simxGetObjectHandle(self.client_id, motorConfig[3], vrep.simx_opmode_blocking)[1], motorConfig[5])
            setattr(SimRobot, motorConfig[3], motor)

            #Sets each motor to streaming opmode
            print("Beginning to stream ", motor.motor_id)

            res = vrep.simx_return_novalue_flag
            while res != vrep.simx_return_ok:
                res = vrep.simxGetJointPosition(self.client_id, motor.handle, vrep.simx_opmode_streaming)[0]
           
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
        return 2

    def get_tf_luna_data(self):
        # dist = float(vrep.simxGetFloatSignal(self.client_id, "proximity", vrep.simx_opmode_streaming)[1])
        # print(dist)
        # if dist > 5:
        #     print("stop")
        return 6

    def get_husky_lens_data(self):
        return 3

    def open_hand(self):
        pass

    def close_hand(self):
        pass

    def stop_hand(self):
        pass


class RealRobot(Robot):

    def __init__(self):
        self.arduino_serial = ArduinoSerial()
        self.motors = self.motors_init()
        print("here")
        self.primitives = []
        self.is_real = True
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
        super().__init__(True, self.motors)

    def motors_init(self):
        motors = list()
        for motorConfig in Config.motors:
            #                    motorID        angleLimit         name              serial
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
                data.append(.000001)
        # self.arduino_serial.send_command('42')  # reads Euler angles
        # euler_angles = self.arduino_serial.read_command()
        # print("Raw Euler angles are " + str(euler_angles))
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

    def open_hand(self):
        self.left_hand_motor.rotation_on(10)

    def close_hand(self):
        self.left_hand_motor.rotation_on(-10)

    def stop_hand(self):
        self.left_hand_motor.rotation_off()
