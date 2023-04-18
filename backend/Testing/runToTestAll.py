from threading import Thread

from backend.Testing.runToTestMovement import setup
from backend.Primitives.MovementManager import play_motion
from backend.Testing.runToTestSensors import pv

robot, file_name, pose_time, pose_delay = setup()
movement_thread = Thread(target=play_motion, args=(robot, file_name, pose_time, pose_delay))
thready = Thread(target=pv.run, args=robot)
