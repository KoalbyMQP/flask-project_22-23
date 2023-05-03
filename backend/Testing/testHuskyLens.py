from threading import Thread
import sys

from backend.KoalbyHumanoid.Robot import RealRobot
# from backend.simulation import sim as vrep
# from backend.KoalbyHumanoid.Robot import SimRobot

robot = RealRobot()  # inits real-world robot

# robot.get_husky_lens_data()

while 1:
    print(robot.get_husky_lens_data())  # gets HuskyLens data

    # thready1 = Thread(target=robot.get_husky_lens_data)
    # thready1.start()