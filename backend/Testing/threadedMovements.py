import sys
import time
import threading
import logging


from backend.Simulation import sim as vrep
from backend.KoalbyHumaniod.Robot import SimRobot

vrep.simxFinish(-1)  # just in case, close all opened connections
client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # inits simulation
if client_id != -1:  # TODO: Fix duplicate code
    print("Connected to remote API server")
else:
    sys.exit("Not connected to remote API server")

robot = SimRobot(client_id)  # inits sim robot


def do_move(joint_name):
    """
    Moves a specified joint in simulation
    :param joint_name: The name of the joint to be moved
    """
    # print("starting thread")
    logging.info("Thread %s: starting", joint_name)

    i = 0
    while i < 100:
        handle = vrep.simxGetObjectHandle(client_id, joint_name, vrep.simx_opmode_blocking)[1]  # gets ID of sim motor
        vrep.simxSetJointTargetPosition(client_id, handle, i, vrep.simx_opmode_streaming)  # set target angle position
        i = i + 10
        time.sleep(.5)
        print("moved {} {} degrees".format(joint_name, i))

    # print("ending thread")
    logging.info("Thread %s: finishing", joint_name)

# UNSURE WHAT IS HAPPENING BELOW HERE, SOMEONE ELSE SHOULD COMMENT

# thready = threading.Thread(target=do_move("Left_Elbow_Joint"))
# thready2 = threading.Thread(target=do_move("Right_Elbow_Joint"))
# thready.start()
# print("here")
# thready2.start()
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO,
                        datefmt="%H:%M:%S")
threads = list()
for index in range(2):
    logging.info("Main    : create and start thread %d.", index)
    name = "Left_Elbow_Joint"
    if index == 1:
        name = "Right_Elbow_Joint"

    x = threading.Thread(target=do_move, args=(name,))

    threads.append(x)
    x.start()

for index, thread in enumerate(threads):
    logging.info("Main    : before joining thread %d.", index)
    thread.join()
    logging.info("Main    : thread %d done", index)



