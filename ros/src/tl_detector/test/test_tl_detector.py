import os, sys
import numpy as np
sys.path.append(os.path.abspath(os.path.join('..')))

import impl.tl_detector as tl_impl

import unittest

class TestTLDetector(unittest.TestCase):
    def __init__(self, methodName = 'runTest'):
        return super().__init__(methodName)

    def test_all_methods_throw_ValueError_if_object_not_initialized(self):
        tld = tl_impl.TLDetector()
        self.assertRaises(ValueError, tld.get_closest_waypoint, [0, 0])

    c_waypoints_2d = {
        'hor_line' : [[0,0], [1,0], [2,0], [3,0]],
        'ver_line' : [[0,0], [0,1], [0,2], [0,3]],
        'diag_line' : [[0,0], [1,1], [2,2], [3,3]],
        'L_line' : [[0,0], [1,0], [1,1]]
    }

    def test_hor_line_get_closest_waypoint_delivers_expected_values(self):
        sut = tl_impl.TLDetector(self.c_waypoints_2d['hor_line'])

        exp = [
            ([0, 0], 0),
            ([-0.1, 42.0], 0),
            ([0.9, 0.2], 1),
            ([2.1, -0.2], 2),
            ([3.5, 0.2], 3)
        ]

        act = []
        for query, idx in exp:
            act.append((query, sut.get_closest_waypoint(query)))

        self.assertEqual(exp, act)

    def test_ver_line_get_closest_waypoint_delivers_expected_values(self):
        sut = tl_impl.TLDetector(self.c_waypoints_2d['ver_line'])

        exp = [
            ([0, 0], 0),
            ([-0.1, 42.0], 3),
            ([0.9, 0.9], 1),
            ([2.1, 1.9], 2),
            ([0, 2.5001], 3)
        ]

        act = []
        for query, idx in exp:
            act.append((query, sut.get_closest_waypoint(query)))

        self.assertEqual(exp, act)

    def test_diag_line_get_closest_waypoint_delivers_expected_values(self):
        sut = tl_impl.TLDetector(self.c_waypoints_2d['diag_line'])

        exp = [
            ([0, 0], 0),
            ([1, 0.5], 1),
            ([1.5, 2], 2),
            ([2.6, 2.6], 3),
            ([4, 4], 3)
        ]

        act = []
        for query, idx in exp:
            act.append((query, sut.get_closest_waypoint(query)))

        self.assertEqual(exp, act)

    def test_L_line_get_closest_waypoint_delivers_expected_values(self):
        sut = tl_impl.TLDetector(self.c_waypoints_2d['L_line'])

        exp = [
            ([0, -1], 0),
            ([1, -1], 1),
            ([2, -1], 1),
            ([4, 4], 2)
        ]

        act = []
        for query, idx in exp:
            act.append((query, sut.get_closest_waypoint(query)))

        self.assertEqual(exp, act)

if __name__ == '__main__':
    unittest.main(verbosity=2)