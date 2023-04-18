import time

from backend.KoalbyHumaniod.Robot import RealRobot

robot = RealRobot()

while True:
    # data = robot.get_husky_lens_data()
    # print(data)
    # print("here")
    # uart = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    data = robot.get_tf_luna_data()
    print(data)
    # if data == '173':
    #     print("calc")
    #     uart[0] = 173
    #     for i in range(1, 9):
    #         uart[i] = int(robot.get_tf_luna_data())
    #
    #     check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7]
    #     # if uart[8] == (check & 0xff):
    #     dist = uart[2] + uart[3] * 256
    #     print(dist)
