import time

from backend.KoalbyHumanoid.Robot import RealRobot
from backend.Primitives.MovementManager import record_motion


def record_movement(robot):
    # robot = RealRobot()  # inits real-world robot
    pose_num = int(input("Input number of poses desired:"))
    time.sleep(3)
    record_motion(robot, pose_num)  # records motor values corresponding to a position


robot = RealRobot()
record_movement(robot)
print("Done")
