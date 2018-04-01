import impl.tl_classifier
import cv2
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    c_map_to_styx_msg = {
        'red': TrafficLight.RED,
        'yellow': TrafficLight.YELLOW,
        'green': TrafficLight.GREEN,
        'none': TrafficLight.UNKNOWN
    }

    def __init__(self, path_to_graph):
        #TODO load classifier
        self.impl = impl.tl_classifier.TLClassifier(path_to_graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return self.c_map_to_styx_msg[self.impl.get_classification(image)]