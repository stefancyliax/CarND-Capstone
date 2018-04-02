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
        'L_line' : [[0,0], [1,0], [1,1]],
        'L_line_long' : [[0,0], [1,0], [1,1], [2,1], [2,2], [3,2]]
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

    def test_L_line_long_closest_traffic_lights_deliver_expected_values(self):
        sut = tl_impl.TLDetector(self.c_waypoints_2d['L_line_long'])

        # mocks the TrafficLight list
        c_lights = [0,
            1,
            2,
            3,
            4
        ]

        # position of stop lines for respective traffic light len(c_lights) == len(c_lights_stop_lines_2d)
        c_lights_stop_lines_2d = [
            [0.25, -0.25],
            [0.75, 0.25],
            [2.25, 0.75],
            [2.25, 1.75],
            [3.25, 1.75]
        ]

        # position, closest wp, light 
        exp = [
            ([0.25, -0.25], 0, 0),
            ([0.75, 0.25], 1, 1),
            ([2.25, 0.75], 3, 2),
            ([2.25, 1.75], 4, 3),
            ([3.25, 1.75], 5, 4)
        ]

        act = []
        for pos_2d, exp_wp_idx, exp_closest_light in exp:
            act_wp_idx, act_closest_light = sut.get_closest_traffic_light(pos_2d, c_lights, c_lights_stop_lines_2d)
            act.append((pos_2d, act_wp_idx, act_closest_light))
        self.assertEqual(exp, act)

    def test_diag_line_closest_traffic_lights_deliver_expected_values(self):
        sut = tl_impl.TLDetector(self.c_waypoints_2d['diag_line'])

        # mocks the TrafficLight list
        c_lights = [
            0,
            1
        ]

        # position of stop lines for respective traffic light len(c_lights) == len(c_lights_stop_lines_2d)
        c_lights_stop_lines_2d = [
            [0, 0],
            [1, 1]
        ]

        # position, closest wp, light 
        exp = [
            ([0, 0], 0, 0),
            ([0.75, 0.75], 1, 1),
            ([2.5, 2.5], -1, None),
            ([3, 3], -1, None)
        ]

        act = []
        for pos_2d, exp_wp_idx, exp_closest_light in exp:
            act_wp_idx, act_closest_light = sut.get_closest_traffic_light(pos_2d, c_lights, c_lights_stop_lines_2d)
            act.append((pos_2d, act_wp_idx, act_closest_light))
        self.assertEqual(exp, act)
    
if __name__ == '__main__':
    unittest.main(verbosity=2)