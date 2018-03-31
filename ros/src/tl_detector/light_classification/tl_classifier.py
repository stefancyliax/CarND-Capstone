from impl import tl_classifier as impl
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    map_to_styx = {
        'red': TrafficLight.RED,
        'yellow': TrafficLight.YELLOW,
        'green': TrafficLight.GREEN,
        'none': TrafficLight.NONE
    }

    def __init__(self, path_to_graph):
        #TODO load classifier
        self.impl = impl.TlClassifier(path_to_graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return s_map_to_styx_msg[self.impl.get_classification(image)]