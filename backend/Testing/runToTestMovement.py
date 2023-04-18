import sys

from backend.KoalbyHumaniod.Robot import RealRobot
from backend.Primitives.MovementManager import play_motion


def setup():
    pose_time = float(input("Enter pose time in seconds: "))
    pose_delay = float(input("Enter delay between pose time in seconds: "))
    file_name = str(input("Input saved file name to play back: "))
    robot = RealRobot()

    # if simulation_flag == 1:
    #     vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot)
    return robot, file_name, pose_time, pose_delay

# TODO: tweaks for some reason when executing movement
robot_type, playback_file, p_time, p_delay = setup()
play_motion(robot_type, playback_file, p_time, p_delay)
print("Done")
