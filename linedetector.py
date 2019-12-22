linimport rospy, time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector:

    def __init__(self, topic):

        self.bridge = CvBridge()
        self.cv_image = np.empty(shape=[0])
        self.value_threshold = 180
        self.image_width = 640
        self.image_middle = 320
        self.image_middle = 320
        self.scan_height = 40
        self.area_width, area_height = 20, 10
        self.roi_vertical_pos = 290  # 310
        self.row_begin = (self.scan_height - area_height) // 2
        self.row_end = self.row_begin + area_height
        self.pixel_cnt_threshold = 0.3 * self.area_width * area_height
        self.detect_node = rospy.Subscriber(topic, Image, self.conv_image)
        self.last_l, self.last_r = 0, 0

    def conv_image(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        frame = self.cv_image[self.roi_vertical_pos:self.roi_vertical_pos + self.scan_height, :]
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        dst = cv2.Canny(blur, 50, 200, None, 3)
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        lines = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

        if lines is not None:
            for i in range(0, len(lines)):
                l = lines[i][0]
                cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 5, cv2.LINE_AA)

        lbound = np.array([0, 0, self.value_threshold], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.uint8)

        hsv = cv2.cvtColor(cdst, cv2.COLOR_BGR2HSV)
        self.bin = cv2.inRange(hsv, lbound, ubound)
        self.view = cv2.cvtColor(self.bin, cv2.COLOR_GRAY2BGR)

    def detect_lines(self):
        # Return positions of left and right lines detected.

        left, right = -1, -1

        for l in range(self.image_middle, self.area_width, -1):
            area = self.bin[self.row_begin:self.row_end, l - self.area_width:l]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                left = l
                break

        for r in range(self.image_middle, self.image_width - self.area_width):
            area = self.bin[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                right = r
                break

        if right > 0 and left > 0 and abs(right - left) < 450:
            if self.last_r == -1:
                right = -1
            elif self.last_l == -1:
                left = -1
	'''
        elif left > 0 and abs(self.last_l - left) > 20:
            left = self.last_l
        elif right > 0 and abs(self.last_r - right) > 20:
            right = self.last_r
	'''

        self.last_l = left
        self.last_r = right

        return left, right

    def show_images(self, left, right):

        if left != -1:
            self.lsquare = cv2.rectangle(self.view,
                                         (left, self.row_begin),
                                         (left - self.area_width, self.row_end),
                                         (0, 255, 0), 3)
        else:
            print("Lost left line")

        if right != -1:
            self.rsquare = cv2.rectangle(self.view,
                                         (right, self.row_begin),
                                         (right + self.area_width, self.row_end),
                                         (0, 255, 0), 3)
        else:
            print("Lost right line")

   	cv2.imshow('title', self.view)

    def QRcode(self):
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        decoded = pyzbar.decode(gray)
        for d in decoded:
            x, y, w, h = d.rect

        barcode_data = d.data.decode("utf-8")
        barcode_type = d.type

        cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

        self.text = '%s (%s)' % (barcode_data, barcode_type)
	print(self.text)
        return self.text




if __name__ == "__main__":
    det = LineDetector()
    time.sleep(1)
    while not rospy.is_shutdown():
        det.show_images(det.detect_lines()[0], det.detect_lines()[1])
