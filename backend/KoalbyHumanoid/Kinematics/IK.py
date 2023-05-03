import sys, os
sys.path.insert(0, '/home/pi/Documents/koalby-humanoid')
from ikpy.chain import Chain
from ikpy.urdf import URDF
from ikpy.utils.geometry import rpy_matrix
from ikpy.urdf.URDF import get_chain_from_joints
from numpy import deg2rad, rad2deg, array, arctan2, sqrt
import os
import numpy as np

import xml.etree.ElementTree as ET
import itertools

##
##
##
##  CURRENTLY NOT USING AT ALL
##
##
##

class IKChain(Chain):
    """ Motors chain used for forward and inverse kinematics.
    This class is based on the IK Chain as defined in the IKPY library (https://github.com/Phylliade/ikpy). It
    provides convenient methods to directly create such a chain directly from a Poppy Creature.
    """

    @classmethod
    def from_poppy_creature(cls, poppy, motors, passiv, tip, reversed_motors=[]):
        """ Creates an kinematic chain from motors of a Poppy Creature.
        :param poppy: PoppyCreature used
        :param list motors: list of all motors that composed the kinematic chain
        :param list passiv: list of motors which are passiv in the chain (they will not move)
        :param list tip: [x, y, z] translation of the tip of the chain (in meters)
        :param list reversed_motors: list of motors that should be manually reversed (due to a problem in the URDF?)
        """
        # This works koalbyURDF = 'C:/Users/raymo/PycharmProjects/koalby-humanoid/Kinematics/Koalby_Humanoid.urdf'
        script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in
        rel_path = "Koalby_Humanoid.urdf"
        koalbyURDF = os.path.join(script_dir, rel_path)

        chain_elements = get_chain_from_joints(koalbyURDF, [m.name for m in motors])

        activ = [False] + [m not in passiv for m in motors] + [True]

        chain = cls.from_urdf_file(koalbyURDF,
                                   base_elements=chain_elements,
                                   last_link_vector=tip,
                                   active_links_mask=activ)

        chain.motors = [getattr(poppy, l.name) for l in chain.links[1:-1]]

        for m, l in zip(chain.motors, chain.links[1:-1]):
            l.bounds = tuple(map(rad2deg, m.angle_limit))

        chain._reversed = array([(-1 if m in reversed_motors else 1) for m in motors])

        return chain

    @property
    def joints_position(self):
        """ Returns the joints position of all motors in the chain (in degrees). """
        return [m.getPosition() for m in self.motors]

    # Transformation matrix M:
    # [[ Rx.x, Ry.x, Rz.x, T.x ],      R = M[:3][:3] is the rotation matrix.
    #  [ Rx.y, Ry.y, Rz.y, T.y ],
    #  [ Rx.z, Ry.z, Rz.z, T.z ],      T = M[:3][3] is the translation matrix. It corresponds
    #  [ 0   , 0   , 0,    1  ]]       to the absolute coordinates of the effector

    @property
    def position(self):
        """ Returns the cartesian position of the end of the chain (in meters). """
        angles = self.convert_to_ik_angles(self.joints_position)
        return self.forward_kinematics(angles)[:3, 3]

    @property
    def orientation(self):
        """ Returns the rotation matrix along X axis (values from -1 to 1). """
        angles = self.convert_to_ik_angles(self.joints_position)
        return self.forward_kinematics(angles)[:3, 0]

    @property
    def pose(self):
        """
        Gives the 4x4 afﬁne transformation matrix of the current position
        *Used for debug*
        :return: 4x4 afﬁne transformation matrix (float)
        """
        angles = self.convert_to_ik_angles(self.joints_position)
        return self.forward_kinematics(angles)

    @property
    def rpy(self):
        """
        Gives the rpy values of the current position
        :return: roll, pitch, yaw (float)
        """
        angles = self.convert_to_ik_angles(self.joints_position)
        R = self.forward_kinematics(angles)
        yaw = arctan2(R[2][1], R[1][1])
        pitch = arctan2(-R[3][1], sqrt(R[3][2] ^ 2 + R[3][3] ^ 2))
        roll = arctan2(R[3][2], R[3][3])
        return roll, pitch, yaw

    def rpy_to_rotation_matrix(self, r, p, y):
        """
        converts rpy to a 3x3 rotation matrix
        :param r: roll (float)
        :param p: pitch (float)
        :param y: yaw (float)
        :return: 3x3 rotation matrix
        """
        return rpy_matrix(r, p, y)

    def goto(self, position, orientation, wait=False, accurate=False):
        """ Goes to a given cartesian position.
        :param list position: [x, y, z] representing the target position (in meters)
        :param list orientation: [Rx.x, Rx.y, Rx.z] transformation along X axis (values from -1 to 1)
        :param bool wait: whether to wait for the end of the move
        :param bool accurate: trade-off between accurate solution and computation time. By default, use the not so
        accurate but fast version.
        """
        # if len(position) != 3:
        #     raise ValueError('Position should be a list [x, y, z]!')
        self._goto(position, orientation, wait, accurate)

    def _goto(self, position, orientation, wait, accurate):
        """ Goes to a given cartesian pose.
        :param matrix position: [x, y, z] representing the target position (in meters)
        :param list orientation: [Rx.x, Rx.y, Rx.z] transformation along X axis (values from -1 to 1)
        :param bool wait: whether to wait for the end of the move
        :param bool accurate: trade-off between accurate solution and computation time. By default, use the not so
        accurate but fast version.
        """
        kwargs = {}
        if not accurate:
            kwargs['max_iter'] = 3

        if orientation is not None:
            shape = array(orientation).shape
            if shape == (3, 3):
                orientation_mode = "all"
            elif shape == (3,):
                orientation_mode = "X"
            else:
                orientation_mode = None
        else:
            orientation_mode = None

        # q0 = self.convert_to_ik_angles(self.joints_position)
        q = self.inverse_kinematics(target_position=position,
                                    target_orientation=orientation,
                                    orientation_mode=orientation_mode,
                                    **kwargs)

        print("Q: ", q)

        joints = self.convert_from_ik_angles(q)

        print("Joints: ", joints)

        last = self.motors[-1]
        for m, pos in list(zip(self.motors, joints)):
            if 'r_' in m.name:
                m.setPositionPos(pos)
            # m.goto_position(pos, duration, wait=False if m != last else wait)

    def convert_to_ik_angles(self, joints):
        """ Convert from poppy representation to IKPY internal representation. """
        if len(joints) != len(self.motors):
            raise ValueError('Incompatible data, len(joints) should be {}!'.format(len(self.motors)))
        #print("Final Check", joints)
        raw_joints = [int(float(j)) for j in joints]
        # raw_joints = [(j + m.offset) * (1 if m.direct else -1) for j, m in zip(joints, self.motors)]

        raw_joints *= self._reversed

        return [0] + [deg2rad(j) for j in raw_joints] + [0]

    def convert_from_ik_angles(self, joints):
        """ Convert from IKPY internal representation to poppy representation. """
        if len(joints) != len(self.motors) + 2:
            raise ValueError('Incompatible data, len(joints) should be {}!'.format(len(self.motors) + 2))

        joints = [rad2deg(j) for j in joints[1:-1]]
        joints *= self._reversed

        '''return [(j * (1 if m.direct else -1)) - m.offset
                        for j, m in zip(joints, self.motors)]'''
        return [(j * 1) - 0
                for j, m in zip(joints, self.motors)]

    def inverse_kinematics(self, target_position=None, target_orientation=None, orientation_mode=None, **kwargs):
        """

        Parameters
        ----------
        target_position: np.ndarray
            Vector of shape (3,): the target point
        target_orientation: np.ndarray
            Vector of shape (3,): the target orientation
        orientation_mode: str
            Orientation to target. Choices:
            * None: No orientation
            * "X": Target the X axis
            * "Y": Target the Y axis
            * "Z": Target the Z axis
            * "all": Target the entire frame (e.g. the three axes) (not currently supported)
        kwargs

        Returns
        -------
        list:
            The list of the positions of each joint according to the target. Note : Inactive joints are in the list.
        """
        frame_target = np.eye(4)

        # Compute orientation
        if orientation_mode is not None:
            if orientation_mode == "X":
                frame_target[:3, 0] = target_orientation
            elif orientation_mode == "Y":
                frame_target[:3, 1] = target_orientation
            elif orientation_mode == "Z":
                frame_target[:3, 2] = target_orientation
            elif orientation_mode == "all":
                frame_target[:3, :3] = target_orientation
            else:
                raise ValueError("Unknown orientation mode: {}".format(orientation_mode))

        # Compute target
        if target_position is None:
            no_position = True
        else:
            no_position = False
            frame_target[:3, 3] = target_position

        #print("Frame", frame_target)

        return self.inverse_kinematics_frame(target=frame_target, orientation_mode=orientation_mode, no_position=no_position, **kwargs)
