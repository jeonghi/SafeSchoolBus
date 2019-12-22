import rospy, time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera:

    def __init__(self, topic):
        self.scan_width, self.scan_height = 200, 90  # 200, 90
        self.bridge = CvBridge()
        self.cv_image = np.empty(shape=[0])
        self.detect_node = rospy.Subscriber('usb_cam/image_raw', Image, self.conv_image)
        self.mask = np.zeros(shape=(self.scan_height, self.scan_width),dtype=np.uint8)
        self.pixel_cnt_threshold = 0.3 * self.scan_width * self.scan_height

    def conv_image(self, data):

        roi_vertical_pos = 200
        h = 480
        w = 640
        roi_horizon_pos = (w - self.scan_width) // 2
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])  # 노란색의 HSV 범위값

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        roi = self.cv_image[roi_vertical_pos:roi_vertical_pos + self.scan_height, roi_horizon_pos: w-roi_horizon_pos]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsv, lower_yellow, upper_yellow, 50)

        # 노란색 비율이 30% 이상일때 과속방지턱이라 여긴다

    def detectBumper(self):
        if cv2.countNonZero(self.mask) > self.pixel_cnt_threshold:
            return True

        return False




