from simple_pid import PID


def do_work(yaw, pitch, roll, robot):
    # PID control

    pitch_pid = PID(.5, 0, 0, setpoint=4.4)
    roll_pid = PID(.5, 0, 0, setpoint=4.4)
    yaw_pid = PID(.5, 0, 0, setpoint=4.4)
    # pitch_pid.output_limits = (-.25, .75)  # Output value will be between 0 and 10

    if not robot.is_real:
        # Don't need PID in simulation - it already has built in PID control
        # handle = vrep.simxGetObjectHandle(client_id, 'Lower_Torso_Front2Back_Joint', vrep.simx_opmode_blocking)[1]
        #
        # motor_pos = vrep.simxGetJointPosition(client_id, handle, vrep.simx_opmode_streaming)[1]
        # print("Motor position is {}".format(motor_pos))
        #
        #
        # vrep.simxSetJointTargetPosition(client_id, handle, control, vrep.simx_opmode_streaming)
        # print("Simulation already has PID")
        pass

    else:
        pose_motor_positions_dict = {}

        print("pitch angle is {}".format(pitch))
        control_pitch = pitch_pid(pitch)
        print("control is {}".format(control_pitch))
        pose_motor_positions_dict[robot.get_motor(21)] = control_pitch

        print("roll angle is {}".format(roll))
        control_roll = roll_pid(pitch)
        print("control is {}".format(control_roll))
        pose_motor_positions_dict[robot.get_motor(9)] = control_roll

        print("yaw angle is {}".format(yaw))
        control_yaw = yaw_pid(yaw)
        print("control is {}".format(control_yaw))
        # pose_motor_positions_dict[robot.get_motor(21)] = control_yaw # IDK if there's a yaw motor

        robot.update_motors(250, pose_motor_positions_dict)
