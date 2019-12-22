import cv2
import numpy as np
from pyzbar import pyzbar
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

#qr코드를 읽고 사용하기 위해 pyzbar를 사용한다.
class QRcode:
    def __init__(self,topic):
        self.bridge = CvBridge()
        self.cv_image = np.empty(shape=[0])
        self.scan_node = rospy.Subscriber(topic, Image, self.QRcode)

    def QRcode(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        #불러온 이미지를 gray 이미지로 바꾸어준다.
        decoded = pyzbar.decode(gray)
        #pyzbar에 있는 decode라는 함수로 gray라는 이미지를 해석한다.
        for d in decoded:
            x, y, w, h = d.rect
        #발견한 qrcode의 왼쪽 상단 모서리, 오른쪽 상단 모서리, 왼쪽 하단 모서리
        #오른쪽 하단 모서리의 위치를 찾는다.

            barcode_data = d.data.decode("utf-8")


        cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        #파란색 사각형을 검출된 qr코드 위에다 그린다.
        self.text = str(barcode_data)
        #바코드의 데이타(slow or stop)을 self.text에 보내준다.
        print(self.text)
        return self.text
        #self.text를 리턴해준다.
