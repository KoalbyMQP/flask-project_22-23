from backend.KoalbyHumaniod.Robot import RealRobot
from backend.Primitives.MovementManager import record_motion


def record_movement():
    robot = RealRobot()  # inits real-world robot
    pose_num = int(input("Input number of poses desired:"))
    record_motion(robot, pose_num)  # records motor values corresponding to a position


record_movement()
print("Done")
