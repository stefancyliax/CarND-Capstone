#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import impl.tl_detector as tl_impl
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        # Empty detector
        self.impl = tl_impl.TLDetector()

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(rospy.get_param("/path_to_graph"))
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        #print("TLDetector is ready: {0}".format(self.impl.is_ready()))

        if not self.impl.is_ready():
            wps = self.waypoints
            wps_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in wps.waypoints]
            self.impl = tl_impl.TLDetector(wps_2d)
            if not self.impl.is_ready():
                raise ValueError("tl_impl.TLDetector could not be initialized!")

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

        if not self.pose:
            raise ValueError("Pose not initialized!")
        
        pos = [self.pose.pose.position.x, self.pose.pose.position.y]
        
        #print("Invoking self.impl.get_closest_waypoint({0})".format(pos))
        wp_idx = self.impl.get_closest_waypoint(pos)
        #print("-> {0}".format(wp_idx))

        return wp_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        #print("Invoking self.light_classifier.get_classification()");
        cls_id = self.light_classifier.get_classification(cv_image)
        #print("-> {0}".format(cls_id))

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #Closest_light has type of TrafficLigth, but it is completely irrelevant what it contains since it is not used anywhere in the algorithm. Let's just treat it as proxy which can be None or not None. It is retrieved from the member self.lights which is set by lights_cb
        light_wp_idx, closest_light = -1, None

        if(self.pose):
            #TODO find the closest visible traffic light (if one exists)

            # List of positions that correspond to the line to stop in front of for a given intersection
            #print("Invoking self.impl.get_closest_traffic_light({0}, {1}, {2})".format([self.pose.pose.position.x, self.pose.pose.position.y], "self.lights", len(self.config['stop_line_positions'])))
            light_wp_idx, closest_light = self.impl.get_closest_traffic_light(
                [self.pose.pose.position.x, self.pose.pose.position.y], 
                self.lights, 
                self.config['stop_line_positions']
            )
            #print("-> {0}, {1}".format(light_wp_idx, True if not closest_light is None else False))

        if closest_light:
            state = self.get_light_state(closest_light)
            return light_wp_idx, state
        else:
            return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
