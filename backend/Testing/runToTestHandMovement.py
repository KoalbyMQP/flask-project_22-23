import time

from backend.KoalbyHumanoid.Robot import RealRobot

robot = RealRobot()  # init real-world robot
robot.close_hand()  # closes gripper
time.sleep(100)
robot.open_hand()  # opens gripper
time.sleep(100)
robot.stop_hand()  # stops gripper?
