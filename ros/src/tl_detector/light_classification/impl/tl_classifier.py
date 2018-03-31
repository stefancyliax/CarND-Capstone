import tensorflow as tf
import numpy as np

class TLClassifier(object):
    s_min_score_thresh = .5
    s_category_index = {
        1 : 'green',
        2 : 'red',
        3 : 'yellow',
        4 : 'none'
    }

    def _init_graph(path_to_graph):
        graph = tf.Graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(path_to_graph, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

        return graph, config

    def _init_tensors(self):
        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

    def __init__(self, *args, **kwargs):
        path_to_graph = kwargs['path_to_graph']
        self.graph, self.config = _init_graph(path_to_graph)
        self.session = tf.Session(graph=self.graph, config=self.config)

        return super().__init__(*args, **kwargs)

    def get_class_name(self, idx):
        return self.s_category_index[idx]

    def get_classification(self, image):
        exp_image = np.expand_dims(image, axis=0)
        
        (boxes, scores, classes, num) = self.session.run([self.detection_boxes,                 self.detection_scores, 
                self.detection_classes, 
                self.num_detections], feed_dict={ self.image_tensor: exp_image})

        boxes, scores, classes, num = np.squeeze(boxes), np.squeeze(scores), np.squeeze(classes).astype(np.int32), num

        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > self.s_min_score_thresh:
                class_name = self.get_class_name(classes[i])

        return class_name