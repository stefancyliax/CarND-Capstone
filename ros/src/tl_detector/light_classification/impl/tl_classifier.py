import tensorflow as tf
import numpy as np

class TLClassifier(object):
    # This is set to a very low value in order to meet the tests induced by test_images_udacity and the "mobile" version of the network.  
    c_min_score_thresh = .35
    c_category_index = {
        1 : 'green',
        2 : 'red',
        3 : 'yellow',
        4 : 'none'
    }

    @staticmethod
    def _init_graph(path_to_graph):
        graph = tf.Graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        # Load graph from ProtoBuf file into default instance which is set to "graph". This is a weird idiom but well, who cares...
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(path_to_graph, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

        return graph, config

    @staticmethod
    def _init_tensors(graph):
        # Retrieve input and output tensors from the graph
        return graph.get_tensor_by_name('image_tensor:0'), [ 
            graph.get_tensor_by_name('detection_boxes:0'), 
            graph.get_tensor_by_name('detection_scores:0'),
            graph.get_tensor_by_name('detection_classes:0'),
            graph.get_tensor_by_name('num_detections:0')
        ]

    def __init__(self, path_to_graph, *args, **kwargs):
        self.graph, self.config = TLClassifier._init_graph(path_to_graph)
        self.image_tensor, self.other_tensors = TLClassifier._init_tensors(self.graph)
        self.session = tf.Session(graph=self.graph, config=self.config)
        

    def _get_class_name(self, idx):
        return self.c_category_index[idx]

    def get_classification(self, image):
        exp_image = np.expand_dims(image, axis=0)
        
        (boxes, scores, classes, num) = self.session.run(self.other_tensors, feed_dict={ self.image_tensor: exp_image} ) 

        boxes, scores, classes, num = np.squeeze(boxes), np.squeeze(scores), np.squeeze(classes).astype(np.int32), num

        class_occurances = {
            'red': 0,
            'yellow': 0,
            'green': 0,
            'none': 0
        }
        for i in range(int(num[0])):
            if self.c_min_score_thresh < scores[i]:
                class_occurances[self._get_class_name(classes[i])] += 1
                #print("{0}: {1} - {2}".format(i, self._get_class_name(classes[i]), scores[i]))
            else:
                break

        class_with_max_occurance = max(class_occurances, key=class_occurances.get)
        occurance = class_occurances[class_with_max_occurance]
        return class_with_max_occurance if occurance > 0 else "none" 