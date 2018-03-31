import os, sys
sys.path.append(os.path.abspath(os.path.join('..')))

import light_classification as lc
import unittest

class TestTLClassifier(unittest.TestCase):
    def setUp(self):
        return super().setUp()

    def tearDown(self):
        return super().tearDown()

    def test_can_instantiate(self):
        cl = lc.impl.TLClassifier('../light_classification/frozen_models/frozen_sim_inception-5000/frozen_inference_graph.pb')
        self.assertTrue(cl != None)

    def test_can_load_sim_inception_model(self):
        return

    def test_can_load_real_inception_model(self):
        return

    def test_can_load_sim_mobile_model(self):
        return

    def test_can_load_real_mobile_model(self):
        return

if __name__ == '__main__':
    unittest.main()
