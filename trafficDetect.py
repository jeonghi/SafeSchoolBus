import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

class TrafficDetect:

    def __init__(self, topic):
        self.bridge = CvBridge()
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.detect_node = rospy.Subscriber(topic, Image, self.makeROI)

    def makeROI(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.roi = self.cam_img[100:300, 500:630]
        self.img_hsv = cv2.cvtColor(self.roi, cv2.COLOR_BGR2HSV)

    def detectTraffic(self):
        lower_red = (0, 20, 20)
        upper_red = (5, 255, 255)
        img_mask_red = cv2.inRange(self.img_hsv, lower_red, upper_red)
        img_result_red = cv2.bitwise_and(self.roi, self.roi, mask=img_mask_red)
        gray = cv2.cvtColor(img_result_red, cv2.COLOR_BGR2GRAY)
        ret, check = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        if cv2.countNonZero(check) >8000:
            return 0

        lower_orange = (5, 30, 30)
        upper_orange = (20, 255, 255)
        img_mask_orange = cv2.inRange(self.img_hsv, lower_orange, upper_orange)
        img_result_orange = cv2.bitwise_and(self.roi, self.roi, mask=img_mask_orange)
        gray = cv2.cvtColor(img_result_orange, cv2.COLOR_BGR2GRAY)
        ret, check = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        if cv2.countNonZero(check) > 5000:
            return 1

        lower_green = (60 - 10, 30, 30)
        upper_green = (60 + 10, 255, 255)
        img_mask_green = cv2.inRange(self.img_hsv, lower_green, upper_green)
        img_result_green = cv2.bitwise_and(self.roi, self.roi, mask=img_mask_green)
        gray = cv2.cvtColor(img_result_green, cv2.COLOR_BGR2GRAY)
        ret, check = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        if cv2.countNonZero(check) > 100:
            return 2
