import math
import time

import numpy

from collections import namedtuple

"""
This module can be used to compute the forward and inverse kinematics for a chain of revolute joints.
It has been largerly inspired by the Matlab Robotics Toolbox.

THE KINEMATICS MAY NEED A LOT OF WORK, BE WARY IF THINGS DON'T LOOK RIGHT, IT COULD BE CODE IF YOU THINK IT'S SOMETHING
WRONG WITH THE ACTUAL ROBOT

"""


class Link(namedtuple('Link', ('theta', 'd', 'a', 'alpha', 'length'))):
    """ Link object as defined by the standard DH representation.

    UNITS FOR THETA AND ALPHA ARE IN RADIANS, NOT DEGREES!!!!!!!!!

    Due to floating point error, values that should be zero like numpy.cos(math.pi/2) will come out as a very very small nonzero number,
    this shouldn't matter too much as the number is basically zero

    This representation is based on the following information:
    :param float theta: angle about previous z from old x to new x
    :param float d: offset along previous z to the common normal
    :param float a: offset along previous x to the common normal
    :param float alpha: angle about common normal, from old z axis to new z axis

    note:: We are only considering revolute joint.

    Please refer to http://en.wikipedia.org/wiki/Denavit-Hartenberg_parameters for more details.

    """

    def get_transformation_matrix(self, theta):
        """
        Computes the homogeneous transformation matrix for this link
        :param theta: Angle that the joint has rotated
        :return: Returns the transformation matrix
        """
        ct = numpy.cos(theta + self.theta)  # cosine theta
        st = numpy.sin(theta + self.theta)  # sine theta
        ca = numpy.cos(self.alpha)  # cosine theta
        sa = numpy.sin(self.alpha)  # sine theta

        return numpy.matrix(((ct, -st * ca, st * sa, self.a * ct),  # transformation matrix for the joint
                             (st, ct * ca, -ct * sa, self.a * st),
                             (0, sa, ca, self.d),
                             (0, 0, 0, 1)))

    def get_length(self):
        """ Returns the length of the link object """
        return self.length


class Chain(namedtuple('Chain', ('links', 'base', 'tool'))):
    """ Chain of Link objects that can be used to perform forward and inverse kinematics.

    :param list links: list of Link that compose the chain
    :param base: the base homogeneous transformation matrix
    :param tool: the end tool homogeneous transformation matrix

    """

    def __new__(cls, links, base=numpy.identity(4), tool=numpy.identity(4)):
        return super(Chain, cls).__new__(cls, links, base, tool)

    def forward_kinematics(self, q):
        """ Computes the homogeneous transformation matrix of the end effector of a kinematic chain.

        :param vector q: vector of the joint angles (theta 1, theta 2, ..., theta n)

        """
        q = numpy.array(q).flatten()

        if len(q) != len(self.links):
            raise ValueError('q must contain as element as the number of links')

        tr = self.base.copy()

        intermediateMatrices = []  # list of each cumulative transformation matrix

        for link, theta in zip(self.links, q):
            tr = tr * link.get_transformation_matrix(theta)  # cumulative transformation matrix

            intermediateMatrices.append(tr)  # add the next intermediate matrix to the list of them

        tr = tr * self.tool
        intermediateMatrices.append(tr)
        return tr, numpy.asarray(intermediateMatrices)  # final matrix, list of intermediate matrices

    def inverse_kinematics_leg(self, end_effector_matrix):
        """ Calculates the inverse kinematics of a vector of task space coordinates (p) and returns
            the corresponding angle values as a vector of the same size (q) for Koalby's leg"""
        l1 = self.links[0].length  # length of thigh
        l2 = self.links[1].length  # length of shin
        l3 = self.links[2].length  # length of foot

        p = get_task_space(end_effector_matrix)  # gets the task space vector from the end effector transformation matrix
        print(p)

        xc = p[0]  # x-coordinate
        yc = p[1]  # y-coordinate
        zc = p[2]  # z-coordinate

        # Equations used to calculate the inverse kinematics of the leg geometrically
        r = math.sqrt(xc ** 2 + yc ** 2)
        s = zc - l1
        d = xc / r
        e = (l2 ** 2 + r ** 2 + s ** 2 - l3 ** 2) / (2 * l2 * math.sqrt(r ** 2 + s ** 2))
        f = math.sqrt(1 - e ** 2)
        g = (l3 ** 2 + l2 ** 2 - (r ** 2 + s ** 2)) / (2 * l2 * l3)
        h = math.sqrt(1 - g ** 2)
        alpha = math.atan2(s, r)
        beta = math.atan2(e, f)
        theta1 = math.atan2(math.sqrt(1 - d ** 2), d)
        theta2 = -(alpha - beta)
        theta3 = -(math.atan2(h, g) - math.pi / 2)

        q = [theta1, theta2, theta3]  # joint space vector corresponding to the position calculated

        return q

    def _jacob0(self, q):  # UNUSED, left from a previous team
        Jn = self._jacobn(q)
        Rn = rotation_from_transf(self.forward_kinematics(q)[0])

        return numpy.concatenate((numpy.concatenate((Rn, numpy.zeros((3, 3))), axis=1),
                                  numpy.concatenate((numpy.zeros((3, 3)), Rn), 1))) * Jn

    def _jacobn(self, q):  # UNUSED, left from a previous team
        q = numpy.array(q).flatten()
        U = self.tool.copy()
        J = numpy.matrix([[]] * 6)
        print(list(reversed(list(zip(self.links, q)))))
        # print(q)

        for link, theta in reversed(list(zip(self.links, q))):
            U = link.get_transformation_matrix(theta) * U

            d = numpy.matrix((-U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3],
                              -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3],
                              -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]))
            delta = U[2, 0:3]

            J = numpy.concatenate((numpy.concatenate((d, delta), axis=1).T, J), axis=1)

        return J


# MARK: - Utility functions

def get_task_space(end_effector_transformation):
    """ Pulls the task space position from the values in the first three rows of the last column
        (known as the p vector) for use in inverse kinematics """

    return end_effector_transformation[0:3, 3]

# UNSURE IF ANY OF THE BELOW FUNCTIONS ARE EVEN BEING USED IN THE CURRENT CODE
def transform_difference(t1, t2):
    t1 = numpy.array(t1)
    t2 = numpy.array(t2)

    return numpy.concatenate(((t2[0:3, 3] - t1[0:3, 3]).reshape(3),
                              0.5 * (numpy.cross(t1[0:3, 0], t2[0:3, 0]) +
                                     numpy.cross(t1[0:3, 1], t2[0:3, 1]) +
                                     numpy.cross(t1[0:3, 2], t2[0:3, 2])).reshape(3)))


def rotation_from_transf(tm):
    return tm[0:3, 0:3]


def translation_from_transf(tm):
    return numpy.array(tm[0:3, 3]).reshape(3)


def components_from_transf(tm):
    return rotation_from_transf(tm), translation_from_transf(tm)


def transf_from_components(R, T):
    return numpy.matrix(numpy.vstack((numpy.hstack((R, T.reshape(3, 1))),
                                      (0, 0, 0, 1))))


def transl(x, y, z):
    M = numpy.matrix(numpy.identity(4))
    M[0:3, 3] = numpy.matrix([x, y, z]).T
    return M


def trotx(theta):
    ct = numpy.cos(theta)
    st = numpy.sin(theta)

    R = numpy.matrix(((1, 0, 0),
                      (0, ct, -st),
                      (0, st, ct)))

    return transf_from_components(R, numpy.zeros(3))


def troty(theta):
    ct = numpy.cos(theta)
    st = numpy.sin(theta)

    R = numpy.matrix(((ct, 0, st),
                      (0, 1, 0),
                      (-st, 0, ct)))

    return transf_from_components(R, numpy.zeros(3))


def trotz(theta):
    ct = numpy.cos(theta)
    st = numpy.sin(theta)

    R = numpy.matrix(((ct, -st, 0),
                      (st, ct, 0),
                      (0, 0, 1)))

    return transf_from_components(R, numpy.zeros(3))
