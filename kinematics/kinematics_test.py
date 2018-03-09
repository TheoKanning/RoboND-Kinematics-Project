import unittest
import kinematics
from collections import namedtuple

TestCase = namedtuple('TestCase', 'name ee_pos ee_angles wc_pos joint_angles')

test_cases = [
    TestCase(
        name='All Zero',
        ee_pos=[2.153, 0.0, 1.9465],
        ee_angles=[0, 0, 0],
        wc_pos=[1.85, 0, 1.946],
        joint_angles=[0, 0, 0, 0, 0, 0]),
    TestCase(
        name='Joint 1',
        ee_pos=[1.8039, 1.175, 1.9465],
        ee_angles=[0, 0, .58],
        wc_pos=[1.55, 1.009, 1.946],
        joint_angles=[.58, 0, 0, 0, 0, 0]),
    TestCase(
        name='Joint 2',
        ee_pos=[1.4871, 0.0, 2.5909],
        ee_angles=[0, -0.432, 0],
        wc_pos=[1.2119, 0, 2.4641],
        joint_angles=[0, -0.43, 0, 0, 0, 0]),
    TestCase(
        name='Random Configuration 1',
        ee_pos=[2.16135, -1.42635, 1.55109],
        ee_angles=[1.654, .490, 0.062],
        wc_pos=[1.89451, -1.44302, 1.69366],
        joint_angles=[-0.65, 0.45, -0.36, 0.95, 0.79, 0.49]),
    TestCase(
        name='Random Configuration 2',
        ee_pos=[1.603, -1.011, 1.599],
        ee_angles=[-1.527, .479, -.937],
        wc_pos=[1.444, -.794, 1.739],
        joint_angles=[-.5, -.15, .28, -.81, .54, -.58])
]


class KinematicsTest(unittest.TestCase):

    def assert_position(self, actual, expected, places=1):
        self.assertAlmostEqual(actual[0], expected[0], places, 'wrong x position')
        self.assertAlmostEqual(actual[1], expected[1], places, 'wrong y position')
        self.assertAlmostEqual(actual[2], expected[2], places, 'wrong z position')

    def assert_angles(self, actual, expected, places=1):
        self.assertAlmostEqual(actual[0], expected[0], places, 'wrong roll angle')
        self.assertAlmostEqual(actual[1], expected[1], places, 'wrong pitch angle')
        self.assertAlmostEqual(actual[2], expected[2], places, 'wrong yaw angle')

    def check_effector_pose(self, case):
        angles = case.joint_angles

        actual_position, actual_orientation = kinematics.get_forward(angles)

        self.assert_position(actual_position, case.ee_pos, places=1)
        self.assert_angles(actual_orientation, case.ee_angles, places=1)

    def check_wrist_center(self, case):
        actual_wc = kinematics.get_wrist_center(case.ee_pos, case.ee_angles)
        self.assert_position(actual_wc, case.wc_pos, places=3)

    def test_forward(self):
        for case in test_cases:
            with self.subTest(case.name):
                self.check_effector_pose(case)

    def test_wrist_center(self):
        for case in test_cases:
            with self.subTest(case.name):
                self.check_wrist_center(case)

    def test_inverse(self):
        for case in test_cases:
            with self.subTest(case.name):
                angles = kinematics.get_inverse(case.ee_pos, case.ee_angles)
                # assert first three angles directly
                for i in range(0, 3):
                    self.assertAlmostEqual(angles[i], case.joint_angles[i], 2, "wrong joint {} angle".format(i + 1))

                calculated_pos, calculated_angle = kinematics.get_forward(angles)
                self.assert_position(calculated_pos, case.ee_pos, places=3)
                self.assert_angles(calculated_angle, case.ee_angles, places=3)

    def test_rotation_matrix(self):
        original_angles = (.5, -.3, .7)
        matrix = kinematics.create_rotation_matrix(original_angles)
        new_angles = kinematics.angles_from_transform(matrix)

        self.assertAlmostEqual(new_angles[0], original_angles[0], 5, 'wrong roll angle')
        self.assertAlmostEqual(new_angles[1], original_angles[1], 5, 'wrong pitch angle')
        self.assertAlmostEqual(new_angles[2], original_angles[2], 5, 'wrong yaw angle')
