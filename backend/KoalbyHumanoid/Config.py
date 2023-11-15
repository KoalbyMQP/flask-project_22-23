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
#   0           1           2           3       4           5       6       7           8
#   [motorID, angleLimit, motorType, jointName, realPID, simPID, twist, linkMass, linkHomeConfig]
    # Right Arm
    [1, [0, 0], 'Herk', 'Right_Shoulder_Rotator_Joint', [1,1,1], [5, 0, 1], [-1, 0, 0, 0, -76, 73], 10.17, [[1, 0, 0, -101.09], [0, 1, 0, 74.53], [0, 0, 1, 67.02], [1, 0, 0, 1]]],
    [2, [0, 0], 'Herk', 'Right_Shoulder_Abductor_Joint', [1,1,1], [5, 0, 1], [0, 0, -1, -73.05, -119.1, 0], 71.23, [[1, 0, 0, -127.29], [0, 1, 0, 72.44], [0, 0, 1, 62.05], [1, 0, 0, 1]]],
    [3, [0, 0], 'Herk', 'Right_Upper_Arm_Rotator_Joint', [1,1,1], [5, 0, 1], [1, 0, 0, 0, 76.65, -72], 206.87, [[1, 0, 0, -229.68], [0, 1, 0, 71.22], [0, 0, 1, 67.37], [0, 0, 0, 1]]],
    [15, [0, 0], 'Herk', 'Right_Elbow_Joint', [1,1,1], [5, 0, 1], [0, 1, 0, -64.63, 0, -247.17], 235.84, [[1,0,0,-330.19], [0,1,0,73.82], [0,0,1,54.34], [0,0,0,1]]],
    [25, [0,0], 'Herk', 'Right_Wrist_Joint', [1,1,1], [5, 0, 1], [0, -1, 0, 53.02, 0, -383.96], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],

    # Left Arm
    [11, [0, 0], 'Herk', 'Left_Shoulder_Rotator_Joint', [1,1,1], [5, 0, 1], [1, 0, 0, 0, 76, -73], 10.17, [[1, 0, 0, 101.09], [0, 1, 0, 74.53], [0, 0, 1, 67.02], [1, 0, 0, 1]]],
    [10, [0, 0], 'Herk', 'Left_Shoulder_Abductor_Joint', [1,1,1], [5, 0, 1], [0, 0, -1, -73.05, 119.1, 0], 71.23, [[1, 0, 0, 127.29], [0, 1, 0, 72.44], [0, 0, 1, 62.05], [1, 0, 0, 1]]],
    [6, [0, 0], 'Herk', 'Left_Upper_Arm_Rotator_Joint', [1,1,1], [5, 0, 1], [-1, 0, 0, 0, -76.65, 72], 206.87, [[1, 0, 0, 229.68], [0, 1, 0, 71.22], [0, 0, 1, 67.37], [0, 0, 0, 1]]],
    [7, [0, 0], 'Herk', 'Left_Elbow_Joint', [1,1,1], [5, 0, 1], [0, 1, 0, -64.63, 0, 247.17], 235.84, [[1,0,0,330.19], [0,1,0,73.82], [0,0,1,54.34], [0,0,0,1]]],
    [26, [0,0], 'Herk', 'Left_Wrist_Joint', [1,1,1], [5, 0, 1], [0, -1, 0], [0, -1, 0, 53.02, 0, -383.96], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],

    # Torso
    [17, [0, 0], 'Herk', 'Lower_Torso_Front2Back_Joint', [1,1,1], [5, 0, 1], [-1, 0, 0, 0, -49.9, -114.02], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],
    [18, [0, 0], 'Herk', 'Chest_Side2Side_Joint', [1,1,1], [5, 0, 1], [0, 0, -1, 0, 0, 0], 129.13, [[1,0,0,1.65], [0,1,0,3.44], [0,0,1,40.50], [0,0,0,1]]],
    #[19, [0, 0], 'Herk', 'Upper_Torso_RotatorJoint'],
    [21, [0, 0], 'Herk', 'Lower_Torso_Side2Side_Joint', [1,1,1], [10.0, 0.0, 50.0], [0, 0, -1, -130.05, 1.2, 0], 258.41, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],
    [23, [0, 0], 'Herk', 'Upper_Torso_Rotator_Joint', [1,1,1], [5, 0, 1], [0, -1, 0, 58.47, 0, -1.2], 47.13, [[1,0,0,0.6], [0,1,0,-85.27], [0,0,1,33.17], [0,0,0,1]]],

    # Right Leg
    [9, [0, 0], 'Herk', 'Right_Thigh_Abductor_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],
    [14, [0, 0], 'Herk', 'Right_Thigh_Rotator_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],
    [31, [0, 0], 'Herk', 'Right_Thigh_Kick_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]], #17
    [12, [0, 0], 'Herk', 'Right_Knee_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]], #18
    [5, [0, 0], 'Herk', 'Right_Ankle_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]], #19

    # Left Leg
    [8, [0, 0], 'Herk', 'Left_Thigh_Abductor_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],
    [4, [0, 0], 'Herk', 'Left_Thigh_Rotator_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],
    [30, [0, 0], 'Herk', 'Left_Thigh_Kick_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],
    [20, [0, 0], 'Herk', 'Left_Knee_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],
    [13, [0, 0], 'Herk', 'Left_Ankle_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],

    # Head
    [23, [0, 0], 'Dyn', 'Neck_Forward2Back_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]],
    [24, [0, 0], 'Dyn', 'Neck_Rotator_Joint', [1,1,1], [5, 0, 1], [], 0, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]]

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

links = [
    #Probably will be deleted. Content moved to motors[]
    #linkName, controlledBy, Mass, HomeConfig(4x4)

    #Right Arm
    ["rightShoulderRotatorLink", motors[0], 10.17, [[1, 0, 0, -101.09], [0, 1, 0, 74.53], [0, 0, 1, 67.02], [1, 0, 0, 1]]],
    ["rightShoulderAbductor", motors[1], 71.23, [[1, 0, 0, -127.29], [0, 1, 0, 72.44], [0, 0, 1, 62.05], [1, 0, 0, 1]]],
    ["rightUpperArmRotator", motors[2], 206.87, [[1, 0, 0, -229.68], [0, 1, 0, 71.22], [0, 0, 1, 67.37], [0, 0, 0, 1]]],
    ["rightElbow", motors[3], 235.84, [[1,0,0,-330.19], [0,1,0,73.82], [0,0,1,54.34], [0,0,0,1]]],
    ["", motors[4], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    
    #Left Arm
    ["", motors[5], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[6], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[7], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[8], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[9], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],

    #Chest
    ["", motors[10], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[11], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[12], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[13], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],

    #Right Leg
    ["", motors[14], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[15], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[16], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[17], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[18], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],

    
    #Left Leg
    ["", motors[19], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[20], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[21], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[22], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[23], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],

    #Head
    ["", motors[24], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]],
    ["", motors[25], 0, [[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]]
]