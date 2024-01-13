import time
import math
from abc import ABC, abstractmethod
from enum import Enum

import backend.KoalbyHumanoid.Config as Config
import modern_robotics as mr
from backend.KoalbyHumanoid.Link import RealLink, SimLink
from backend.KoalbyHumanoid.Limb import RealLimb, SimLimb
from backend.KoalbyHumanoid.PID import PID
from backend.ArduinoSerial import ArduinoSerial
from backend.KoalbyHumanoid.Motor import RealMotor, SimMotor
from backend.Simulation import sim as vrep
from backend.KoalbyHumanoid.Sensors.PiratedCode import Kalman_EKF as KM
from backend.Testing import poe as poe

global prevAngleError

class Joints(Enum):
    Right_Shoulder_Rotator_Joint = 0
    Right_Shoulder_Abductor_Joint = 1
    Right_Upper_Arm_Rotator_Joint = 2
    Right_Elbow_Joint = 3
    Right_Wrist_Joint = 4

    # Left Arm
    Left_Shoulder_Rotator_Joint = 5
    Left_Shoulder_Abductor_Joint = 6
    Left_Upper_Arm_Rotator_Joint = 7
    Left_Elbow_Joint = 8
    Left_Wrist_Joint = 9

    # Torso
    Lower_Torso_Front2Back_Joint = 10
    Chest_Side2Side_Joint = 11
    Lower_Torso_Side2Side_Joint = 12
    Upper_Torso_Rotator_Joint = 13

    # Right Leg
    Right_Thigh_Abductor_Joint = 15
    Right_Thigh_Rotator_Joint = 16
    Right_Thigh_Kick_Joint = 17
    Right_Knee_Joint = 18
    Right_Ankle_Joint = 19

    # Left Leg
    Left_Thigh_Abductor_Joint = 20
    Left_Thigh_Rotator_Joint = 21
    Left_Thigh_Kick_Joint = 22
    Left_Knee_Joint = 23
    Left_Ankle_Joint = 24

    # Head
    Neck_Forward2Back_Joint = 25
    Neck_Rotator_Joint = 26 

class Robot(ABC):
    def __init__(self, is_real, motors):
        self.motors = motors
        print("Robot Created and Initialized")
        self.is_real = is_real
        self.sys = KM.System()
        self.prevTime = time.perf_counter()

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
        self.gyro = self.gyro_init()
        self.CoM = 0
        super().__init__(False, self.motors)
        self.primitives = []
        self.is_real = False
        self.chain = self.chain_init()
        self.links = self.links_init()
        self.limbs = self.limbs_init()
        self.PID = PID(0.25,0.1,0)

        # Placeholder need to make this set up the links
    def chain_init(self):
        chain = {
        self.motors[24].name:self.motors[23],
        self.motors[23].name:self.motors[22],
        self.motors[22].name:self.motors[21],
        self.motors[21].name:self.motors[20],
        self.motors[20].name:self.motors[10],

        self.motors[19].name:self.motors[18],
        self.motors[18].name:self.motors[17],
        self.motors[17].name:self.motors[16],
        self.motors[16].name:self.motors[15],
        self.motors[15].name:self.motors[10],

        self.motors[10].name:self.motors[13],
        self.motors[13].name:self.motors[12],
        self.motors[12].name:self.motors[14],
        self.motors[14].name:self.motors[11],
        self.motors[11].name:"base",

        self.motors[4].name:self.motors[3],
        self.motors[3].name:self.motors[2],
        self.motors[2].name:self.motors[1],
        self.motors[1].name:self.motors[0],
        self.motors[0].name:"base",

        self.motors[9].name:self.motors[8],
        self.motors[8].name:self.motors[7],
        self.motors[7].name:self.motors[6],
        self.motors[6].name:self.motors[5],
        self.motors[5].name:"base"
        }
        return chain
    
    def limbs_init(self):
        limbs = list()
        for limbsConfig in Config.limbs:
            limb = SimLimb(limbsConfig[1])
            limbs.append(limb)
        return limbs
    
    def links_init(self):
        links = list()
        for linksConfig in Config.links:
            link = SimLink(linksConfig[0], linksConfig[1])
            links.append(link)
        return links

    def motors_init(self):
        motors = list()
        for motorConfig in Config.motors:
            #if motorConfig[0] == 19: ## Motor does not exist in CoppeliaSim but does exist in Config.py. I am hesitant to delete it, so this is a bandaid fix. -Scott
            #    continue
            handle = vrep.simxGetObjectHandle(self.client_id, motorConfig[3], vrep.simx_opmode_blocking)[1]
            vrep.simxSetObjectFloatParameter(self.client_id, handle, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
            motor = SimMotor(motorConfig[0], self.client_id, handle, motorConfig[5], motorConfig[6], motorConfig[7])
            setattr(SimRobot, motorConfig[3], motor)

            #Sets each motor to streaming opmode
            print("Beginning to stream ", motor.motor_id)
            res = vrep.simx_return_novalue_flag
            while res != vrep.simx_return_ok:
                res = vrep.simxGetJointPosition(self.client_id, motor.handle, vrep.simx_opmode_streaming)[0]
            motor.theta = motor.get_position()
            motor.name = motorConfig[3]
            motors.append(motor)
        return motors

    def gyro_init(self):
        print("Getting IMU Data...")
        vrep.simxGetFloatSignal(self.client_id, "gyroX", vrep.simx_opmode_streaming)[1]
        vrep.simxGetFloatSignal(self.client_id, "gyroY", vrep.simx_opmode_streaming)[1]
        vrep.simxGetFloatSignal(self.client_id, "gyroZ", vrep.simx_opmode_streaming)[1]
        res = vrep.simx_return_novalue_flag
        while res != vrep.simx_return_ok:
            res = vrep.simxGetFloatSignal(self.client_id, "gyroZ", vrep.simx_opmode_streaming)[1]


    def update_motors(self, pose_time_millis, motor_positions_dict):
        """
        Take the primitiveMotorDict and send the motor values to the robot
        """

        for key, value in motor_positions_dict.items():
            for motor in self.motors:
                if str(motor.motor_id) == str(key):
                    motor.set_position(value, self.client_id)

    def updateRobotCoM(self):
        rightArm = self.updateRightArmCoM()
        leftArm = self.updateLeftArmCoM()
        torso = self.updateTorsoCoM()
        rightLeg = self.updateRightLegCoM()
        leftLeg = self.updateLeftLegCoM()
        chestMass = 618.15
        chest = [0, 71.83, 54.35]
        rightArmMass = 524.11
        leftArmMass = 524.11
        torsoMass = 434.67
        rightLegMass = 883.81
        leftLegMass = 889.11
        massSum = rightArmMass+leftArmMass+torsoMass+chestMass
        CoMx = rightArm[0] * rightArmMass + leftArm[0] * leftArmMass + torso[0]*torsoMass + chest[0]*chestMass + rightLeg[0]*rightLegMass + leftLeg[0]*leftLegMass
        CoMy = rightArm[1] * rightArmMass + leftArm[1] * leftArmMass + torso[1]*torsoMass + chest[1]*chestMass + rightLeg[1]*rightLegMass + leftLeg[1]*leftLegMass
        CoMz = rightArm[2] * rightArmMass + leftArm[2] * leftArmMass + torso[2]*torsoMass + chest[2]*chestMass + rightLeg[2]*rightLegMass + leftLeg[2]*leftLegMass
        self.CoM = [CoMx / massSum, CoMy / massSum, CoMz / massSum]
        return self.CoM

    def updateRightArmCoM(self):
        motorList = [self.motors[0], self.motors[1], self.motors[2], self.motors[3], self.motors[4]]
        linkList = [self.links[0], self.links[1], self.links[2], self.links[3], self.links[4]]
        return poe.calcLimbCoM(motorList, linkList)
    
    def updateLeftArmCoM(self):
        motorList = [self.motors[5], self.motors[6], self.motors[7], self.motors[8], self.motors[9]]
        linkList = [self.links[5], self.links[6], self.links[7], self.links[8], self.links[9]]
        return poe.calcLimbCoM(motorList, linkList)
    
    def updateTorsoCoM(self):
        motorList = [self.motors[11], self.motors[13], self.motors[10], self.motors[12], self.motors[14]]
        linkList = [self.links[11], self.links[13], self.links[10], self.links[12], self.links[14]]
        return poe.calcLimbCoM(motorList, linkList)
    
    def updateRightLegCoM(self):
        motorList = [self.motors[15], self.motors[16], self.motors[17], self.motors[18], self.motors[19]]
        linkList = [self.links[15], self.links[16], self.links[17], self.links[18], self.links[19]]
        #print(poe.calcLegCoM(self, motorList))
        return poe.calcLegCoM(self, motorList, linkList)

    def updateLeftLegCoM(self):
        motorList = [self.motors[20], self.motors[21], self.motors[22], self.motors[23], self.motors[24]]
        linkList = [self.links[20], self.links[21], self.links[22], self.links[23], self.links[24]]
        #print(poe.calcLegCoM(self, motorList))
        return poe.calcLegCoM(self, motorList, linkList)


    def shutdown(self):
        vrep.simxStopSimulation(self.client_id, vrep.simx_opmode_oneshot)

    def get_imu_data(self):
        data = [vrep.simxGetFloatSignal(self.client_id, "gyroX", vrep.simx_opmode_buffer)[1],
                vrep.simxGetFloatSignal(self.client_id, "gyroY", vrep.simx_opmode_buffer)[1],
                vrep.simxGetFloatSignal(self.client_id, "gyroZ", vrep.simx_opmode_buffer)[1]]
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
    
    def moveAllTo(self, position):
        for motor in self.motors:
            motor.move(position)
            
    def moveAllToTarget(self):
        for motor in self.motors:
            motor.move(motor.target)

    def locate(self, motor):
        slist = []
        thetaList = []
        M = motor.M
        slist.append(motor.twist)
        thetaList.append(motor.theta)
        next = self.chain[motor.name]
        while next != "base":
            slist.append(next.twist)
            thetaList.append(next.theta)
            next = self.chain[next.name]            
        slist.reverse()
        thetaList.reverse()
        # print(thetaList)
        location = mr.FKinSpace(M,slist,thetaList)
        return location[0:3,3]
    
    def locatePolygon(self):
        slist = []
        thetaList = []
        locations = []
        rightAnkleM = [[1,0,0,-43.49],[0,1,0,659.84],[0,0,1,70.68],[1,0,0,0]]
        leftAnkleM = [[1,0,0,43.49],[0,1,0,659.84],[0,0,1,70.68],[1,0,0,0]]
        rightAnkleMotor = self.motors[Joints.Right_Ankle_Joint.value]
        leftAnkleMotor = self.motors[Joints.Left_Ankle_Joint.value]
        Ms = [rightAnkleM, leftAnkleM]
        ankleMotors = [rightAnkleMotor, leftAnkleMotor]
        for i in range(len(ankleMotors)):
            motor = ankleMotors[i]
            M = Ms[i]
            slist.append(motor.twist)
            thetaList.append(motor.theta)
            next = self.chain[motor.name]
            while next != "base":
                slist.append(next.twist)
                thetaList.append(next.theta)
                next = self.chain[next.name]         
            slist.reverse()
            thetaList.reverse()
            # print(thetaList)
            locations.append(mr.FKinSpace(M,slist,thetaList)[0:3,3])
        return locations
    
    def findSupportPolygon(self):
        rightAnkle = self.locate(self.motors[Joints.Right_Ankle_Joint.value])
        leftAnkle = self.locate(self.motors[Joints.Left_Ankle_Joint.value])
        rightSole = rightAnkle*rightAnkleToSole #-43.49,659.84,70.68
        leftSole = leftAnkle*leftAnkleToSole #-43.49,659.84,70.68
        rightPolyCoords = rightSole[0:3,4]
        leftPolyCoords = leftSole[0:3,4]

    def IMUBalance(self, Xtarget, Ztarget):
        pitch = self.get_imu_data()[0]
        roll = self.get_imu_data()[1]
        Xerror = math.radians(Xtarget - pitch)
        Zerror = math.radians(Ztarget - roll)
        self.motors[14].target += -(Xerror)
        self.motors[10].target += (Xerror)
        self.motors[11].target += (Zerror)

    def balanceAngle(self):
        # targetZ = 88
        # zError = targetZ - self.CoM[2]
        # target = 0.11
        # self.locate(self.motors[Joints.Left_Ankle_Joint.value])*ankleL_to_sole
        staticCoM = [-5.7919215528026395, -478.1476301728874, 77.13638385800057]
        staticKickLoc = [93.54, -209.39, 41.53]
        targetTheta = math.atan2(staticCoM[1] - staticKickLoc[1], staticCoM[2] - staticKickLoc[2])
        kickMotorPos = self.locate(self.motors[Joints.Left_Thigh_Kick_Joint.value])
        currTheta = math.atan2(self.CoM[1] - kickMotorPos[1], self.CoM[2] - kickMotorPos[2])
        thetaError = targetTheta - currTheta
        # print(math.degrees(targetTheta), math.degrees(currTheta), thetaError, self.CoM)
        # print(self.CoM)
        # print(math.degrees(self.motors[22].target), math.degrees(self.motors[22].target), math.degrees(self.motors[17].target), math.degrees(self.motors[19].target))
        self.PID.setError(thetaError)
        newTarget = self.PID.calculate()
        
        self.motors[22].target = -newTarget
        self.motors[24].target = -newTarget
        self.motors[17].target = newTarget
        self.motors[19].target = newTarget
        self.IMUBalance(0, 0)
        return thetaError
    
    def balanceZ(self):
        self.IMUBalance()
        # targetZ = 88
        # zError = targetZ - self.CoM[2]
        # target = 0.11
        staticCoM = [-5.7919215528026395, -478.1476301728874, 77.13638385800057]
        targetZ = staticCoM[2]
        curZ = self.CoM[2]
        staticKickLoc = [93.54, -209.39, 41.53]
        targetTheta = math.atan2(staticCoM[1] - staticKickLoc[1], staticCoM[2] - staticKickLoc[2])
        kickMotorPos = self.locate(self.motors[Joints.Left_Thigh_Kick_Joint.value])
        currTheta = math.atan2(self.CoM[1] - kickMotorPos[1], self.CoM[2] - kickMotorPos[2])
        thetaError = targetTheta - currTheta
        # print(math.degrees(targetTheta), math.degrees(currTheta), thetaError, self.CoM)
        # print(self.CoM)
        # print(math.degrees(self.motors[22].target), math.degrees(self.motors[22].target), math.degrees(self.motors[17].target), math.degrees(self.motors[19].target))
        self.motors[22].target = -thetaError
        self.motors[24].target = -thetaError
        self.motors[17].target = thetaError
        self.motors[19].target = thetaError
        return thetaError
        
    def IK(self, motor, T, thetaGuess):
        """Computes the Inverse Kinematics from the Body Frame to the desired end effector motor

        Args:
            eeMotor (SimMotor): Motor you want to calculate IK towards
            T (4x4 Matrix): The desired final 4x4 matrix depicting the final position and orientation
        """
        Slist = []
        Slist.append(motor.twist)
        next = self.chain[motor.name]
        while next != "base":
            Slist.append(next.twist)
            next = self.chain[next.name]
        Slist.reverse()
        M = motor.home
        eomg = 0.01
        ev = 0.01
        return (mr.IKinSpace(Slist, M, T, thetaGuess, eomg, ev))
        


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
