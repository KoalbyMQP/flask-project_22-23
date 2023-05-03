import time

from backend.KoalbyHumanoid.Sensors.PiratedCode.BoardDisplay_EKF import initializeCube, ProjectionViewer
from backend.KoalbyHumanoid.Robot import RealRobot

# Port Finder
# import serial.tools.list_ports as ports
#
# com_ports = list(ports.comports())  # create a list of com ['COM1','COM2']
# for i in com_ports:
#     print(i.device)  # returns 'COMx' / current COM port

robot = RealRobot()  # inits real-world robot
while True:
    robot.arduino_serial.send_command('42')
    print(robot.arduino_serial.read_command())  # reads IMU data
    time.sleep(.5)

    # robot.arduino_serial.send_command('41')
    # data = robot.get_imu_data()
    # # print(data)
    # if data is not None:
    #     if len(data) != 9:
    #         continue
    #     print(robot.get_filtered_data(data))
    # time.sleep(.25)

# print(robot.get_imu_data())
# while True:
#     data = robot.get_imu_data()
#     print(data)
#
#     if data is not None:
#         if len(data) != 9:
#             continue
#         print(robot.get_filtered_data(data))
#     time.sleep(.25)

# block = initializeCube()  # UNSURE WHAT THIS DOES SOMEONE COMMENT THIS
# print("before viewer")
# pv = ProjectionViewer(640, 480, block)
# print("This will go on forever. Simulation and code needs to be manually stopped")
# pv.run(robot, "", 1)
