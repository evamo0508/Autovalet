#!/usr/bin/env python


import sys
import os


import cv2
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


import rospy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from img_processing import pipeline, undistort



class Image_Saver:
	def __init__(self):
		self.bag_sub  = rospy.Subscriber("/realsense/frontCamera/color/image_raw",Image,self.bag_img_callback)
		# self.current_feed_sub = rospy.Subscriber("/subbu", Image, self.current_img_callback)

		self.bridge = CvBridge()
		self.i = 2000
		self.save_flag = False
		self.first = False

	def bag_img_callback(self,data):
		# bag images will end in 0
		print(data.header.stamp.secs)

		if data.header.stamp.secs % 2 == 0 and self.first == False:
			self.save_flag = True
			img = self.bridge.imgmsg_to_cv2(data,"bgr8")
			cv2.imwrite('/home/kob51/Desktop/green_left/train_image{:>05}.jpg'.format(self.i), img)
			self.i +=1
			self.first = True
		else:
			self.first = False

	def current_img_callback(self, data):
		# current images will end in 1
		if self.save_flag:
			self.save_flag = False
			img = self.bridge.imgmsg_to_cv2(data,"bgr8")
			cv2.imwrite('/home/kob51/Desktop/images1/test_image{:>05}.jpg'.format(self.i), img)
			self.i += 1


if __name__ == "__main__":

	rospy.init_node('test')

	detector = Image_Saver()

	rospy.spin()