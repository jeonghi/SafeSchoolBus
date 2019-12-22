#!/usr/bin/env python

import rospy, time
from linedetector import LineDetector
from motordriver import MotorDriver
from obstacledetector import ObstacleDetector
from trafficDetect import TrafficDetect
from speedBump import SpeedBump
from QRcode import QRcode

class AutoDrive:

	def __init__(self):
		rospy.init_node('xycar_driver')
		self.line_detector = LineDetector('/usb_cam/image_raw')
		self.driver = MotorDriver('/xycar_motor_msg')
		self.obstacle_detector = ObstacleDetector('/ultrasonic')
		self.traffic = TrafficDetect('usb_cam/image_raw')
		self.bump = SpeedBump()
		self.code = QRcode('usb_cam/image_raw')

	def trace(self):
		obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
		line_l, line_r = self.line_detector.detect_lines()
		self.line_detector.show_images(line_l, line_r)
		angle = self.steer(line_l, line_r)
		speed = self.accelerate(angle, obs_l, obs_m, obs_r)
		if speed == 0:
			angle = 0
		self.driver.drive(angle + 90, speed + 90)

	# self.driver.drive(90,90)

	def steer(self, left, right):
		print(left, right)
		if left == -1:
			if 320 < right < 390:
				angle = -50
			else:
				angle = (550 - right)/(-3)
		elif right == -1:
			if 250 < left < 320:
				angle = 50
			else:
				angle = (left - 90)/(3)
		else:
			angle = 0

		return angle

	def accelerate(self, angle, obs_l, obs_m, obs_r):
		traffic_color = self.traffic.detectTraffic()
		# 받아온 신호등 값
		text = self.code.QRcode()
		# 받아온 QR코드 값
		bump = self.bump.getStatus()
		# 받아온 방지턱 값
		QRcnt = 0
		# QR코드로 "slow"를 받을 떄 다시 전진 할 수 있도록 변수를 줬다.
		speed = 0
		if text == "slow":
			speed = 30
		# "slow"를 받아오면 speed값을 30으로 받아온다.
		elif text == "stop":
			speed = 0
		# "stop"을 받아오면 speed값을 0으로 받아오고
			QRcnt += 1
			# QRcnt 값을 1씩 더한다.
			# QRcnt 값이 40이 됬을 경우 text값을 "slow"로 줘서 다시 전진하도록 한다
			if QRcnt == 40:
				text = "slow"
		if bump == True:
			speed = 20
		#방지턱을 인식하면 20으로 감속한다.
		elif bump == False:
		#방지턱을 인식하지 않았을 때
			if traffic_color == 0:
				print("red")
				speed = 0
			# 신호등 값이 0이면 빨간 불이므로 정지한다.
			elif traffic_color == 1:
				print("orange")
				speed = 20
			#신호등 값이 1이면 주항 불이므로 감속한다.
			else:
				speed = 30
		# 그 외의 상황은 어린이 보호 구역이 아니므로
		# 빠른 속도로 주행하도록 한다.
		else:
			if 0 < obs_m < 50 and 0 < angle:
				speed = 0
			elif angle <= -40 or angle >= 40:
				speed = 35
			elif angle <= -25 or angle >= 25:
				speed = 45
			else:
				speed = 50

		return speed

	def exit(self):
		print('finished')

if __name__ == '__main__':
	car = AutoDrive()
	time.sleep(1)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		car.trace()
	rate.sleep()
	rospy.on_shutdown(car.exit)