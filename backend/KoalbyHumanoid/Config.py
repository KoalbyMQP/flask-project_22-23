#!/usr/bin/env python3
"""
Motor 1 - Herkulex, Right Forearm
Motor 2 - Herkulex, Right Upper Arm
Motor 3 - Herkulex, Right Arm Connector
Motor F - Herkulex, Right Shoulder

Motor B - Herkulex, Left Forearm
Motor A - Herkulex, Left Upper Arm
Motor 6 - Herkulex, Left Arm Connector
Motor 7 - Herkulex, Left Shoulder

Motor 11 - Herkulex, Torso Double Rotation Backside
Motor 12 - Herkulex, Torso Double Rotation Frontside
Motor 13 - Herkluex, Abdomen
"""

'''Array of all motors for Koalby.'''
motors = [
#   [motorID, angleLimit, motorType, jointName, realPID, simPID]
    # Right Arm
    [1, [0, 0], 'Herk', 'Right_Shoulder_Rotator_Joint', [1,1,1], [1, 0, 10]],
    [2, [0, 0], 'Herk', 'Right_Shoulder_Abductor_Joint', [1,1,1], [1, 0, 10]],
    [3, [0, 0], 'Herk', 'Right_Upper_Arm_Rotator_Joint', [1,1,1], [1, 0, 10]],
    [15, [0, 0], 'Herk', 'Right_Elbow_Joint', [1,1,1], [1, 0, 10]],

    # Left Arm
    [11, [0, 0], 'Herk', 'Left_Shoulder_Rotator_Joint', [1,1,1], [1, 0, 10]],
    [10, [0, 0], 'Herk', 'Left_Shoulder_Abductor_Joint', [1,1,1], [1, 0, 10]],
    [6, [0, 0], 'Herk', 'Left_Upper_Arm_Rotator_Joint', [1,1,1], [1, 0, 10]],
    [7, [0, 0], 'Herk', 'Left_Elbow_Joint', [1,1,1], [1, 0, 10]],

    # Torso
    [17, [0, 0], 'Herk', 'Lower_Torso_Front2Back_Joint', [1,1,1], [1, 0, 10]],
    [18, [0, 0], 'Herk', 'Chest_Side2Side_Joint', [1,1,1], [1, 0, 10]],
    [19, [0, 0], 'Herk', 'Upper_Torso_RotatorJoint'],
    [21, [0, 0], 'Herk', 'Lower_Torso_Side2Side_Joint', [1,1,1], [8, 0, 10]],
    [23, [0, 0], 'Herk', 'Upper_Torso_Rotator_Joint', [1,1,1], [1, 0, 10]],

    # Right Leg
    [9, [0, 0], 'Herk', 'Right_Thigh_Abductor_Joint', [1,1,1], [1, 0, 10]],
    [14, [0, 0], 'Herk', 'Right_Thigh_Rotator_Joint', [1,1,1], [1, 0, 10]],
    [31, [0, 0], 'Herk', 'Right_Thigh_Kick_Joint', [1,1,1], [1, 0, 10]],
    [12, [0, 0], 'Herk', 'Right_Knee_Joint', [1,1,1], [1, 0, 10]],
    [5, [0, 0], 'Herk', 'Right_Ankle_Joint', [1,1,1], [1, 0, 10]],

    # Left Leg
    [8, [0, 0], 'Herk', 'Left_Thigh_Abductor_Joint', [1,1,1], [1, 0, 10]],
    [4, [0, 0], 'Herk', 'Left_Thigh_Rotator_Joint', [1,1,1], [1, 0, 10]],
    [30, [0, 0], 'Herk', 'Left_Thigh_Kick_Joint', [1,1,1], [1, 0, 10]],
    [20, [0, 0], 'Herk', 'Left_Knee_Joint', [1,1,1], [1, 0, 10]],
    [13, [0, 0], 'Herk', 'Left_Ankle_Joint', [1,1,1], [1, 0, 10]],

    # Head
    [23, [0, 0], 'Dyn', 'Neck_Forward2Back_Joint', [1,1,1], [1, 0, 10]],
    [24, [0, 0], 'Dyn', 'Neck_Rotator_Joint', [1,1,1], [1, 0, 10]]


    # # Right Arm
    # [3, [0, 0], 'Herk', 'Right_Shoulder_Rotator_Joint'],
    # [2, [0, 0], 'Herk', 'Right_Shoulder_Abductor_Joint'],
    # [1, [0, 0], 'Herk', 'Right_Upper_Arm_Rotator_Joint'],
    # [0, [0, 0], 'Herk', 'Right_Elbow_Joint'],
    #
    # # Left Arm
    # [7, [0, 0], 'Herk', 'Left_Shoulder_Rotator_Joint'],
    # [6, [0, 0], 'Herk', 'Left_Shoulder_Abductor_Joint'],
    # [5, [0, 0], 'Herk', 'Left_Upper_Arm_Rotator_Joint'],
    # [4, [0, 0], 'Herk', 'Left_Elbow_Joint'],
    #
    # # Torso
    # [21, [0, 0], 'Herk', 'Lower_Torso_Front2Back_Joint'],
    # [22, [0, 0], 'Herk', 'Chest_Side2Side_Joint'],
    # [10, [0, 0], 'Herk', 'Upper_Torso_RotatorJoint'],
    # [9, [0, 0], 'Herk', 'Lower_Torso_Side2Side_Joint'],
    # [8, [0, 0], 'Herk', 'Upper_Torso_Rotator_Joint'],
    #
    # # Right Leg
    # [16, [0, 0], 'Herk', 'Right_Thigh_Abductor_Joint'],
    # [17, [0, 0], 'Herk', 'Right_Thigh_Rotator_Joint'],
    # [30, [0, 0], 'Herk', 'Right_Thigh_Kick_Joint'],
    # [19, [0, 0], 'Herk', 'Right_Knee_Joint'],
    # [20, [0, 0], 'Herk', 'Right_Ankle_Joint'],
    #
    # # Left Leg
    # [11, [0, 0], 'Herk', 'Left_Thigh_Abductor_Joint'],
    # [12, [0, 0], 'Herk', 'Left_Thigh_Rotator_Joint'],
    # [31, [0, 0], 'Herk', 'Left_Thigh_Kick_Joint'],
    # [14, [0, 0], 'Herk', 'Left_Knee_Joint'],
    # [15, [0, 0], 'Herk', 'Left_Ankle_Joint'],
    #
    # # Head
    # [23, [0, 0], 'Dyn', 'Neck_Forward2Back_Joint'],
    # [24, [0, 0], 'Dyn', 'Neck_Rotator_Joint']
]

# THIS DICTIONARY IS UNUSED FROM LAST YEAR'S TEAMs
motorDict = {
    0x01: [[0, 0], 'Herk', 'Right Forearm'],
    0x02: [[0, 0], 'Herk', 'Right Upper Shoulder'],
    0x03: [[0, 0], 'Herk', 'Right Arm Connector'],
    0x0F: [[0, 0], 'Herk', 'Right Shoulder'],

    0x0B: [[0, 0], 'Herk', 'Left Forearm'],
    0x0A: [[0, 0], 'Herk', 'Left Upper Shoulder'],
    0x06: [[0, 0], 'Herk', 'Left Arm Connector'],
    0x07: [[0, 0], 'Herk', 'Left Shoulder'],

    0x11: [[0, 0], 'Herk', 'Torso Double Rotation Back'],
    0x12: [[0, 0], 'Herk', 'Torso Double Rotation Front'],
    0x13: [[0, 0], 'Herk', 'Abdomen']
}

''' Array of all motor groups for Koalby.'''
motorGroups = [
    ['r_arm', motors[0:4]],
    ['l_arm', motors[4:8]],
    ['torso', motors[8:13]],
    ['r_leg', motors[13:20]]]