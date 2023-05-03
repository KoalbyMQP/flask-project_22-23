# import null as null
import json
import threading

from flask import Flask, Response, request
from flask import render_template
from serial import SerialException

from backend.KoalbyHumanoid.Robot import RealRobot, SimRobot
from backend.KoalbyHumanoid.Sensors.sensorData import SensorData
from backend.Primitives import MovementManager
from backend.Simulation import sim as vrep
from backend.Testing.runToTestWalk import Walker

app = Flask(__name__)

robot = None
sensor_data = None
client_id = -1
walker = None
hand = None


@app.route("/")
def welcome():
    return render_template("welcome.html")


@app.route("/init")
def init():
    """
    When Initialize button is pressed from UI, this method is called.
    Initializes robot and all sensors
    :return: 1 if robot is not connected, 0 if robot is
    """
    global robot
    global walker
    walker = Walker()
    walker.init_walk(1)  # 1 is leg choice. Not giving this choice to the user
    sensor_data = SensorData()
    try:
        robot = RealRobot()
    except SerialException:
        return Response("0", mimetype="text/xml")

    sensor_data.init_robot(robot)
    return Response("1", mimetype="text/xml")


@app.route("/shutoff")
def shutdown():
    """
    Shuts robot off when shutdown button is pressed
    Should only be available to press if robot is connected
    :return: 1 when robot has successfully been shutdown
    """
    global robot
    robot.shutdown()  # works
    return Response("1", mimetype="text/xml")


@app.route("/init-simulation")
def init_simulation():
    """
    Initializes simulation when Simulation button is pressed from UI
    IMPORTANT: In order for this to work, the simulation needs to be running
    :return: 0 if not connected to simulation, 1 if connected
    """
    global robot
    global sensor_data
    sensor_data = SensorData()
    global client_id
    global walker
    walker = Walker()
    walker.init_walk(1)  # 1 is leg choice. Not giving this choice to the user

    # TODO: this is duplicated lots of places
    vrep.simxFinish(-1)  # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    print(client_id)  # if 1, then we are connected.
    if client_id != -1:
        print("Connected to remote API server")
    else:
        # sys.exit("Not connected to remote API server")
        print("not connected")
        return Response("0", mimetype="text/xml")

    robot = SimRobot(client_id)
    print("robot should be created")
    sensor_data.init_robot(robot)
    return Response("1", mimetype="text/xml")


@app.route("/home/")
def home():
    return render_template("home.html")


# @app.route("/home/pre-recorded/queue")
# def queue():
#     return render_template("queue.html")


@app.route("/run", methods=['POST'])
def run():
    """
    When run button is clicked after editing queue, this method will be called
    The queue is sent as a JSON object, then parsed and executed
    :return: finished if movements were successfully executed, failure if not
    """
    global client_id
    content_type = request.headers.get('Content-Type')
    if content_type == 'application/json':
        q_list = request.get_json()
        # TODO: make sure the 1s aren't hard coded
        MovementManager.split_list(robot, q_list, 1, 1)  # parses JSON then executes queue
        return Response("finished", mimetype="text/xml")
    # return failure
    return Response("failure", mimetype="text/xml")


@app.route("/pre-recorded/")
def pre_recorded():
    return render_template("pre-recorded.html")


@app.route("/record-new/")
def record_new():
    return render_template("record-new.html")


@app.route("/sensor-data-page/")
def sensor_data_page():
    return render_template("sensor-data.html")


@app.route("/record-one-new/", methods=['POST'])
def record_one_new():
    """
    When record movement is pressed, this gets called
    Records poese one by one and writes to filename each time
    :return: Finished Recording if record was successful, failure if not
    """
    global client_id
    content_type = request.headers.get('Content-Type')
    if content_type == 'application/json':
        file_and_first = request.get_json()
        file_name = file_and_first["fileName"]
        first = file_and_first["firstTime"]
        MovementManager.record_motion_ui(robot, file_name, first)
        return Response("Finished Recording", mimetype="text/xml")
    # return failure
    return Response("failure", mimetype="text/xml")


@app.route("/sensor-data/")
def sensor_data():
    """
    When view sensor data button is clicked, this method is called
    Will auto-populate data chart
    :return: JSON of sensor data
    """
    data = sensor_data.get_data()
    json_object = json.dumps(data)  # formats into actual JSON object
    return Response(json_object, mimetype="text/xml")


# Need this for threading walk
threads = []
stop_flags = []


@app.route("/walk-toggle/")
def walk_toggle():
    """
    Sets the isWalking property to ture, making the robot start walking
    :return: Robot finished walking to the user. Will only return after robot stops walking
    """
    global walker
    if not walker.is_walking:
        stop_flags.append(False)
        thread = threading.Thread(target=walker.play, args=(1, 1, robot, stop_flags, len(stop_flags) - 1,))
        threads.append(thread)
        thread.start()
        walker.is_walking = True
        return Response("Robot began walking", mimetype="text/xml")
    else:
        stop_flags[-1] = True
        walker.is_walking = False
        return Response("Robot stopped walking", mimetype="text/xml")
    # walker.toggle("walk", 1, 1, robot)


@app.route("/open-hand/")
def open_hand():
    """
    Turns the left hand motor to open the grip
    :return: Opening Hand to the user
    """
    robot.open_hand()
    return Response("Opening Hand", mimetype="text/xml")


@app.route("/open-hand/")
def stop_hand():
    """
    Stops the left hand motor from rotating
    :return: Stopping hand to the user
    """
    robot.stop_hand()
    return Response("Stopping Hand", mimetype="text/xml")


@app.route("/close-hand/")
def close_hand():
    """
    Turns the left hand motor to close the grip
    :return: Closing Hand to the user
    """
    robot.close_hand()
    return Response("Closing Hand", mimetype="text/xml")


@app.route("/is-real")
def realHuh():
    real_flag = robot.is_real
    print(real_flag)
    json_object = json.dumps({"isReal": real_flag})  # formats into actual JSON object
    return Response(json_object, mimetype="text/xml")


if __name__ == '__main__':
    app.run(host='172.20.10.2', port=5000, debug=True, threaded=False)
    # app.run(host='192.168.1.148', port=5000, debug=True, threaded=False)  #IP address here
    # app.run(host='0.0.0.0', port=5000, debug=True, threaded=False)
