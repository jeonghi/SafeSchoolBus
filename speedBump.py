import rospy
from diagnostic_msgs.msg import DiagnosticArray
from camera import Camera


class ImuRead:

    def __init__(self, topic):
        self.roll = -1
        self.pitch = -1
        self.raw = -1
        rospy.Subscriber(topic, DiagnosticArray, self.read_data)

    def read_data(self, data):
        status = data.status[0].values
        self.roll = status[0].value
        self.pitch = status[1].value
        self.yaw = status[2].value

    def get_data(self):
        # return float(self.roll), float(self.fitch), float(self.yaw)
        return float(self.fitch)


class SpeedBump:

    def __init__(self):
        self.status = False
        self.pitch = -1
        self.imu = ImuRead('')
        self.checkUp = False
        self.checkDown = False
        self.checkNormal = False

    def isBump(self):
        if self.status == False:
            ### 여기서 roi에서 노흰 비율 학인하고
            switch = Camera('usb_cam/image_raw').detectBumper()

            if switch:
                self.status = True
                self.pitch = self.imu.get_data()
        else:
            if self.imu.get_data() > self.pitch + 8:
                self.checkUp = True

            if self.imu.get_data() < self.pitch - 9:
                self.checkDown = True

            if self.checkUp and self.checkDown:
                if self.pitch - 2 < self.imu.get_data() < self.pitch + 2:
                    self.checkNormal = True
                if self.checkNormal:
                    self.resetData()

    def resetData(self):
        self.checkUp = False
        self.checkDown = False
        self.checkNormal = False
        self.status = False
        self.pitch = -1

    def getStatus(self):
        self.isBump()
        return self.status
