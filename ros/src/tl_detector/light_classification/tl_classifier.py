import cv2
import numpy as np
from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
import operator


class TLClassifier(object):
    def __init__(self):
        self.debug_traffic_image_pub = rospy.Publisher('/debug_traffic_image', Image, queue_size=1)
        self.bridge = CvBridge()
        return

    def get_classification(self, image, bb_list, is_simulator_mode):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light
            bb_list (List): List containing bounding boxe(s) of Traffic Lights

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # if bb_list is empty, return UNKNOWN
        if not bb_list:
            return TrafficLight.UNKNOWN
        else:
            # Step. get highest probability of bb_list, and crop image
            xmin = bb_list[0].xmin
            xmax = bb_list[0].xmax
            ymin = bb_list[0].ymin
            ymax = bb_list[0].ymax

            bb_image = image[ymin:ymax, xmin:xmax]

            # Step. Put crop image and get light state
            if int(is_simulator_mode) == 1:
                # simulator
                return self.detect_light_state_simulator(bb_image)
            else:
                # site
                return self.detect_light_state_site(bb_image)

    def detect_light_state_simulator(self, bb_image):

        # Calculate bb_image pixels of color via hsv space.

        # Step. convert to hsv space.
        hsv_bb_img = cv2.cvtColor(bb_image, cv2.COLOR_BGR2HSV)

        # Reference from https://stackoverflow.com/questions/42882498/what-are-the-ranges-to-recognize-different-colors-in-rgb-space
        # Red color
        hsv_red1_range = cv2.inRange(hsv_bb_img, (0 / 360 * 255, 70, 50), (20/360 * 255, 255, 255))
        hsv_red2_range = cv2.inRange(hsv_bb_img, (340.0 / 360 * 255, 70, 50), (360.0/360 * 255, 255, 255))

        # Yellow color
        hsv_yellow_range = cv2.inRange(hsv_bb_img, (40.0 / 360 * 255, 100, 100), (70.0 / 360 * 255, 255, 255))

        # Green color
        hsv_green_range = cv2.inRange(hsv_bb_img, (90.0 / 360 * 255, 100, 100), (140.0 / 360 * 255, 255, 255))


	    # Debug trffic image
        self.debug_traffic_image_pub.publish(self.bridge.cv2_to_imgmsg(hsv_bb_img, "bgr8"))

        PIXEL_THRESHOLD = 50
        # Red range in hsv space that two region ,(0~20) and (340~360)degree.
        if (cv2.countNonZero(hsv_red1_range) + cv2.countNonZero(hsv_red2_range))/2 > PIXEL_THRESHOLD:
            print('Red Light')
            return TrafficLight.RED
        elif cv2.countNonZero(hsv_yellow_range) > PIXEL_THRESHOLD:
            print('Yellow Light')
            return TrafficLight.YELLOW
        elif cv2.countNonZero(hsv_green_range) > PIXEL_THRESHOLD:
            print('Green Light')
            return TrafficLight.GREEN
        else:
            print('Unknow')
            return TrafficLight.UNKNOWN

    def detect_light_state_site(self, bb_image):

        height, width, channels = bb_image.shape

        # Step. Split traffic light to 3 region(red, yellow and green)
        red_region = bb_image[0:height//3, 0:width]
        yellow_region = bb_image[height//3: 2*height//3, 0:width]
        green_region = bb_image[2*height//3: height, 0:width]

        # Coefficients(bgr) = [0.114, 0.587, 0.299]
	    # Reference : https://en.wikipedia.org/wiki/Grayscale
        coef_red = [0.075, 0.075, 0.85]
        coef_yellow = [0.114, 0.587, 0.299]
        coef_green = [0.075, 0.85, 0.075]

        # Step. get grayscale image.
        red_region = cv2.transform(red_region, np.array(coef_red).reshape((1,3)))
        yellow_region = cv2.transform(yellow_region, np.array(coef_yellow).reshape((1,3)))
        green_region = cv2.transform(green_region, np.array(coef_green).reshape((1,3)))

        # Step . Generate a greyscale traffic light image
        bb_image = np.concatenate((red_region,yellow_region,green_region),axis=0)

        height, width = bb_image.shape

        bb_mask = np.zeros((height, width), np.uint8)

        offset = 3

        cv2.ellipse(bb_mask, (width//2, 1*height//6), (width//2 - offset, height//6 - offset), 0, 0, 360, 1, -1)
        cv2.ellipse(bb_mask, (width//2, 3*height//6), (width//2 - offset, height//6 - offset), 0, 0, 360, 1, -1)
        cv2.ellipse(bb_mask, (width//2, 5*height//6), (width//2 - offset, height//6 - offset), 0, 0, 360, 1, -1)


        bb_image = np.multiply(bb_image, bb_mask)

        # Step . under grey threshold 210 to 255, count pixel of each region
        bb_image = cv2.inRange(bb_image, 210, 255)

        red_region = bb_image[0:height//3, 0:width]
        yellow_region = bb_image[height//3: 2*height//3, 0:width]
        green_region = bb_image[2*height//3: height, 0:width]

        red_count = cv2.countNonZero(red_region)
        yellow_count = cv2.countNonZero(yellow_region)
        green_count = cv2.countNonZero(green_region)

        # Debug trffic image
        self.debug_traffic_image_pub.publish(self.bridge.cv2_to_imgmsg(bb_image, "mono8"))

        s={}
        s[TrafficLight.RED]=red_count
        s[TrafficLight.YELLOW]=yellow_count
        s[TrafficLight.GREEN]=green_count

        # Step. get the light state that max pixel of region.
        state = max(s.iteritems(), key=operator.itemgetter(1))[0]

        if red_count==0 and yellow_count==0 and green_count==0:
            print("unknow")
            state = TrafficLight.UNKNOWN
        else:
            if state == TrafficLight.RED:
                print ('Red Light')
            elif state == TrafficLight.YELLOW:
                print ('Yellow Light')
            elif state == TrafficLight.GREEN:
                print ('Green Light')

        return state