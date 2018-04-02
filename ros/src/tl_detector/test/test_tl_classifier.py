import os, sys
sys.path.append(os.path.abspath(os.path.join('..')))

import light_classification.impl.tl_classifier as lc_impl

import tensorflow as tf
import cv2
import unittest

def get_models_dir():
    return os.path.join(os.path.dirname(__file__), "..", "light_classification", "frozen_models")

def get_images_dir(image_dir_name):
    return os.path.join(get_models_dir(), image_dir_name)

def get_model_path(model_name):
    return os.path.join(get_models_dir(), model_name, "frozen_inference_graph.pb")

def get_image_path(image_dn, image_fn):
    return os.path.join(get_images_dir(image_dn), image_fn)

# I simply assume that the nets expect a RGB image. I use cv2 because this is closer to what happens in tl_detector.py
def get_image_as_cv2rgb(image_path):
    return cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)

class TestTLClassifier(unittest.TestCase):
    c_models = {
        "sim_inception": get_model_path("frozen_sim_inception-5000"),
        "real_inception": get_model_path("frozen_real_inception-5000"),
        "sim_mobile": get_model_path("frozen_sim_mobile-5000"),
        "real_mobile": get_model_path("frozen_real_mobile-5000"),
    }

    c_exp_sim_classes = {
        "left0003.jpg":"green",
        "left0011.jpg":"yellow",
        "left0027.jpg":"red",
        "left0034.jpg":"red",
        "left0036.jpg":"red",
        "left0040.jpg":"red", # this is questionable should be "none"
        "left0048.jpg":"none",
        "left0545.jpg":"green",
        "left0560.jpg":"green",
        "left0588.jpg":"yellow",
        "left0606.jpg":"red",
        "left0607.jpg":"red"
    }

    c_exp_udacity_classes = {
        "left0000.jpg":"green",
        "left0140.jpg":"red",
        "left0183.jpg":"red",
        "left0282.jpg":"green",
        "left0358.jpg":"green",
        "left0528.jpg":"red",
        "left0561.jpg":"red",
        "left0681.jpg":"green",
        "left0701.jpg":"yellow"
    }

    def setUp(self):
        self.tf_logging_verb = tf.logging.get_verbosity()
        tf.logging.set_verbosity(tf.logging.FATAL)
        return super().setUp()

    def tearDown(self):
        tf.logging.set_verbosity(self.tf_logging_verb)
        return super().tearDown()

    def test_can_load_sim_inception_model(self):
        cl = lc_impl.TLClassifier(self.c_models["sim_inception"])
        self.assertTrue(cl != None)
        self.assertTrue(cl.graph != None)
        self.assertTrue(cl.config != None)
        self.assertTrue(cl.image_tensor != None)
        self.assertTrue(cl.other_tensors != None and len(cl.other_tensors) ==
        4)

    def test_sim_image_loader_works_as_expected(self):
        # Using "sorted" to achieve predictable ordering
        for img_fn, exp_img_class in sorted(self.c_exp_sim_classes.items()):
            img_path = get_image_path("test_images_sim", img_fn)
            self.assertTrue(os.path.isfile(img_path))
            img = get_image_as_cv2rgb(img_path)
            self.assertIsNotNone(img)

    def test_udacity_image_loader_works_as_expected(self):
        for img_fn, exp_img_class in sorted(self.c_exp_udacity_classes.items()):
            img_path = get_image_path("test_images_udacity", img_fn)
            self.assertTrue(os.path.isfile(img_path))
            img = get_image_as_cv2rgb(img_path)
            self.assertIsNotNone(img)
            
    def test_sim_inception_model_classifies_test_images_as_expected(self):
        cl = lc_impl.TLClassifier(self.c_models["sim_inception"])
        act_img_classes = {}
        for img_fn, exp_img_class in sorted(self.c_exp_sim_classes.items()):
            print(img_fn)
            img = get_image_as_cv2rgb(get_image_path("test_images_sim", img_fn))
            act_img_classes[img_fn] = cl.get_classification(img)
        self.maxDiff = None
        self.assertDictEqual(self.c_exp_sim_classes, act_img_classes)

    def test_real_inception_model_classifies_test_images_as_expected(self):
        cl = lc_impl.TLClassifier(self.c_models["real_inception"])
        act_img_classes = {}
        for img_fn, exp_img_class in sorted(self.c_exp_udacity_classes.items()):
            print(img_fn)
            img = get_image_as_cv2rgb(get_image_path("test_images_udacity", img_fn))
            act_img_classes[img_fn] = cl.get_classification(img)
        self.maxDiff = None
        self.assertDictEqual(self.c_exp_udacity_classes, act_img_classes)

    def test_sim_mobile_model_classifies_test_images_as_expected(self):
        cl = lc_impl.TLClassifier(self.c_models["sim_mobile"])
        act_img_classes = {}
        for img_fn, exp_img_class in sorted(self.c_exp_sim_classes.items()):
            print(img_fn)
            img = get_image_as_cv2rgb(get_image_path("test_images_sim", img_fn))
            act_img_classes[img_fn] = cl.get_classification(img)
        self.maxDiff = None
        self.assertDictEqual(self.c_exp_sim_classes, act_img_classes)
   
    def test_real_mobile_model_classifies_test_images_as_expected(self):
        cl = lc_impl.TLClassifier(self.c_models["real_mobile"])
        act_img_classes = {}
        for img_fn, exp_img_class in sorted(self.c_exp_udacity_classes.items()):
            print(img_fn)
            img = get_image_as_cv2rgb(get_image_path("test_images_udacity", img_fn))
            act_img_classes[img_fn] = cl.get_classification(img)
        self.maxDiff = None
        self.assertDictEqual(self.c_exp_udacity_classes, act_img_classes)

if __name__ == '__main__':
    unittest.main(verbosity=2)