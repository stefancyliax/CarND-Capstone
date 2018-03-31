import os, sys
sys.path.append(os.path.abspath(os.path.join('..')))

import light_classification.impl.tl_classifier as lc_impl

import unittest

def get_models_dir():
    return os.path.join(os.path.dirname(__file__), "..", "light_classification", "frozen_models")

def get_model_path(model_name):
    return os.path.join(get_models_dir(), model_name, "frozen_inference_graph.pb")

class TestTLClassifier(unittest.TestCase):
    c_models = {
        "sim_inception": get_model_path("frozen_sim_inception-5000"),
        "real_inception": get_model_path("frozen_real_inception-5000"),
        "sim_mobile": get_model_path("frozen_sim_mobile-5000"),
        "real_mobile": get_model_path("frozen_real_mobile-5000"),
    }

    def setUp(self):
        return super().setUp()

    def tearDown(self):
        return super().tearDown()

    def test_can_load_sim_inception_model(self):
        cl = lc_impl.TLClassifier(self.c_models["sim_inception"])
        self.assertTrue(cl != None)
        self.assertTrue(cl.graph != None)
        self.assertTrue(cl.config != None)
        self.assertTrue(cl.image_tensor != None)
        self.assertTrue(cl.other_tensors != None and len(cl.other_tensors) == 4)

    def test_can_load_real_inception_model(self):
        cl = lc_impl.TLClassifier(self.c_models["real_inception"])
        self.assertTrue(cl != None)
        self.assertTrue(cl.graph != None)
        self.assertTrue(cl.config != None)
        self.assertTrue(cl.image_tensor != None)
        self.assertTrue(cl.other_tensors != None and len(cl.other_tensors) == 4)

    def test_can_load_sim_mobile_model(self):
        cl = lc_impl.TLClassifier(self.c_models["sim_mobile"])
        self.assertTrue(cl != None)
        self.assertTrue(cl.graph != None)
        self.assertTrue(cl.config != None)
        self.assertTrue(cl.image_tensor != None)
        self.assertTrue(cl.other_tensors != None and len(cl.other_tensors) == 4)


    def test_can_load_real_mobile_model(self):
        cl = lc_impl.TLClassifier(self.c_models["real_mobile"])
        self.assertTrue(cl != None)
        self.assertTrue(cl.graph != None)
        self.assertTrue(cl.config != None)
        self.assertTrue(cl.image_tensor != None)
        self.assertTrue(cl.other_tensors != None and len(cl.other_tensors) == 4)

if __name__ == '__main__':
    unittest.main()