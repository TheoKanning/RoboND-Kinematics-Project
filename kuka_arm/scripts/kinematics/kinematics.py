import numpy as np
from sympy import symbols, cos, sin, pi, simplify, Transpose
from math import atan2, sqrt, acos
from sympy.matrices import Matrix

# Symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# DH Parameters
s = {alpha0:     0, a0:      0, d1:  0.75,
     alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
     alpha2:     0, a2:   1.25, d3:     0,
     alpha3: -pi/2, a3: -0.054, d4:  1.50,
     alpha4:  pi/2, a4:      0, d5:     0,
     alpha5: -pi/2, a5:      0, d6:     0,
     alpha6:     0, a6:      0, d7: 0.303, q7: 0}

q = symbols('q')
a = symbols('a')
alpha = symbols('alpha')
d = symbols('d')

BASE_T = Matrix([[cos(q), -sin(q), 0, a],
                 [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                 [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                 [0, 0, 0, 1]])

T0_1 = BASE_T.subs({alpha: alpha0, a: a0, d: d1, q: q1}).subs(s)
T1_2 = BASE_T.subs({alpha: alpha1, a: a1, d: d2, q: q2}).subs(s)
T2_3 = BASE_T.subs({alpha: alpha2, a: a2, d: d3, q: q3}).subs(s)
T3_4 = BASE_T.subs({alpha: alpha3, a: a3, d: d4, q: q4}).subs(s)
T4_5 = BASE_T.subs({alpha: alpha4, a: a4, d: d5, q: q5}).subs(s)
T5_6 = BASE_T.subs({alpha: alpha5, a: a5, d: d6, q: q6}).subs(s)
T6_G = BASE_T.subs({alpha: alpha6, a: a6, d: d7, q: q7}).subs(s)

# Gripper link orientation correction
R_z = Matrix([[cos(np.pi), -sin(np.pi), 0, 0],
              [sin(np.pi), cos(np.pi), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

R_y = Matrix([[cos(-np.pi/2), 0, sin(-np.pi/2), 0],
              [0, 1, 0, 0],
              [-sin(-np.pi/2), 0, cos(-np.pi/2), 0],
              [0, 0, 0, 1]])

R_corr = R_z * R_y
R_corr3 = R_corr[:3, :3]


def get_forward(angles):
    """
    Calculates EE position and orientation given joint angles
    returns ([x, y, z], [x, y, z, w])
    """
    transform = T0_1.evalf(subs={q1: angles[0]})
    transform *= T1_2.evalf(subs={q2: angles[1]})
    transform *= T2_3.evalf(subs={q3: angles[2]})
    transform *= T3_4.evalf(subs={q4: angles[3]})
    transform *= T4_5.evalf(subs={q5: angles[4]})
    transform *= T5_6.evalf(subs={q6: angles[5]})
    transform *= T6_G
    transform *= R_corr.evalf()

    rotation = angles_from_transform(transform)

    x = transform[0, 3]
    y = transform[1, 3]
    z = transform[2, 3]

    return [x, y, z], rotation


def angles_from_transform(transform):
    """
    Returns the euler angles corresponding to the given 3x3 rotation matrix
    :param transform: 4x4 Matrix object
    :return: (roll, pitch, yaw)
    """
    matrix = transform.evalf()
    r11 = matrix[0, 0]
    r21 = matrix[1, 0]
    r31 = matrix[2, 0]
    r32 = matrix[2, 1]
    r33 = matrix[2, 2]

    yaw = atan2(r21, r11)
    pitch = atan2(-r31, sqrt(r11**2 + r21**2))
    roll = atan2(r32, r33)

    return roll, pitch, yaw


def wrist_angles_from_transform(transform):
    """
    Returns the necessary wrist joint angles to create the required transformation
    :param transform: Matrix object
    :return: theta4, theta5, theta6
    """
    matrix = transform.evalf()
    r13 = matrix[0, 2]
    r21 = matrix[1, 0]
    r22 = matrix[1, 1]
    r23 = matrix[1, 2]
    r33 = matrix[2, 2]

    # calculated by me
    theta4 = atan2(r33, -r13)
    theta5 = atan2(sqrt(r22 ** 2 + r21 ** 2), r23)
    theta6 = atan2(-r22, r21)

    return theta4, theta5, theta6


def create_rotation_matrix(angles):
    """
    Returns a rotation matrix that will produce the given Euler angles
    :param angles: (roll, pitch, yaw)
    """
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]]).evalf(subs={q: angles[0]})

    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]]).evalf(subs={q: angles[1]})

    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]]).evalf(subs={q: angles[2]})

    return R_z * R_y * R_x


def get_wrist_center(ee_position, ee_orientation):
    """
    Return the wrist center for a given ee position and orientation
    """
    gripper_position = Matrix(ee_position)
    r0_6 = create_rotation_matrix(ee_orientation)

    # d7 is distance from joint 6 to gripper
    displacement = r0_6 * R_corr3 * Matrix([[0], [0], [d7]]).evalf(subs=s)
    wc_position = gripper_position - displacement

    return wc_position


def get_first_three_joints(wrist_pos):
    """
    Calculate angles of the first three joints for the given wrist position
    :param wrist_pos: (x, y, z)
    :return: (joint1, joint2, joint3)
    """
    # project onto xy plane
    theta1 = atan2(wrist_pos[1], wrist_pos[0])

    def law_of_cosines(a, b, c):
        """
        Return angle given lengths of sides a, b, and c
        """
        return acos((c ** 2 - a ** 2 - b ** 2) / (-2 * a * b))

    link2 = s[a2]
    link3 = sqrt(s[a3]**2 + s[d4]**2)
    # wrist position projected onto plane of arm, minus joint 2 position
    projected_wc = sqrt(wrist_pos[0]**2 + wrist_pos[1]**2) - s[a1], wrist_pos[2] - s[d1]
    d2_wc = sqrt(projected_wc[0]**2 + projected_wc[1]**2)

    # theta 2 is equal to the triangle angle plus the angle of the wc
    theta2 = law_of_cosines(link2, d2_wc, link3) + atan2(projected_wc[1], projected_wc[0])
    theta2 = -theta2 + np.pi / 2  # correct for join 2 orientation

    # add correction because link is not straight
    theta3 = - law_of_cosines(link2, link3, d2_wc) + np.pi/2 + atan2(s[a3], s[d4])

    return theta1, theta2, theta3


def get_inverse(position, orientation):
    """
    Given the desired position and orientation of the end effector, calculate the required joint angles
    :param position:
    :param orientation:
    :return:
    """
    wc = get_wrist_center(position, orientation)
    theta1, theta2, theta3 = get_first_three_joints(wc)

    r0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={q1: theta1, q2: theta2, q3: theta3})[:3, :3]
    r0_6 = create_rotation_matrix(orientation) * R_corr3
    r3_6 = Transpose(r0_3) * r0_6
    theta4, theta5, theta6 = wrist_angles_from_transform(r3_6)
    return theta1, theta2, theta3, theta4, theta5, theta6

