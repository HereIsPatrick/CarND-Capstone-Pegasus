#!/usr/bin/env python
from scipy.spatial import KDTree
import math
import numpy as np
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
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

        # waypoint(kd tree)
        self.waypoints_xy = None
        self.waypoint_kd_tree = None
        self.bb_list = None

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
        sub4 = rospy.Subscriber('/image_color', Image, self.image_cb)
        
        # darknet_ros message
        sub5 = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # Is simulator mode
        self.is_simulator_mode = rospy.get_param("/is_simulator_mode")

        # Step. public stop waypoint index.
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # Debug
        #self.debug_traffic_image_pub = rospy.Publisher('/debug_traffic_image', Image, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
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
        # Step. use kd tree to enhance waypoint search.
        if not self.waypoints_xy:
            self.waypoints_xy = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_kd_tree = KDTree(self.waypoints_xy)


    def traffic_cb(self, msg):
        self.lights = msg.lights


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
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



    def bounding_boxes_cb(self, msg):

        self.bb_list = []

        size_thresh_simulator = 80 #px
        size_thresh_site = 40 #px

        prob_thresh_simulator = 0.85 #%
        prob_thresh_site = 0.25 #%

        if int(self.is_simulator_mode) == 1:
            prob_thresh = prob_thresh_simulator
            size_thresh = size_thresh_simulator
        else:
            prob_thresh = prob_thresh_site
            size_thresh = size_thresh_site

        # Step . check traffic light.
        for box in msg.bounding_boxes:
            if str(box.Class) == 'traffic light' and box.probability >= prob_thresh:
                if math.sqrt((box.xmin - box.xmax)**2 + (box.ymin - box.ymax)**2) >= size_thresh:
                    self.bb_list.append(box)

                    if int(self.is_simulator_mode) == 0:
                        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                        bb_image = cv_image[box.ymin:box.ymax, box.xmin:box.xmax]
                        self.light_classifier.detect_light_state_site(bb_image)


    def get_closest_waypoint(self, x, y):
        return self.waypoint_kd_tree.query([x, y], 1)[1] #return closest waypoint index.


    def get_light_state(self, light):

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        return self.light_classifier.get_classification(cv_image, self.bb_list, self.is_simulator_mode)

    def process_traffic_lights(self):
        closest_light = None
        line_waypoint_index = None
        is_light_close = False


        # Get stop line position from dataset
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose and self.waypoints):
            # Step . get closest waypoint of current car
            current_waypoint_index = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            total_waypoint = len(self.waypoints.waypoints)

            diff = total_waypoint

            #print("Lights :", self.lights)

            # Step . find closest light
            for i, light in enumerate(self.lights):

                light_closest_waypoint_index = self.get_closest_waypoint(stop_line_positions[i][0], stop_line_positions[i][1])

                # Step. get left waypoint of light.
                left_waypoint = light_closest_waypoint_index - current_waypoint_index

                if left_waypoint >= 0 and left_waypoint < diff:
                    diff = left_waypoint
                    closest_light = light
                    line_waypoint_index = light_closest_waypoint_index
                    is_light_close = True
                    print("Total waypoint : ", total_waypoint, " next light waypoint_index :", line_waypoint_index, " Left light waypoint : ", left_waypoint )

        if is_light_close==True:
            # Have closest light
            return line_waypoint_index, self.get_light_state(closest_light)
        else:
            # No closest light
            return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

