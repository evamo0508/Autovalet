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



class LaneDetector:
	def __init__(self):
		self.im_sub  = rospy.Subscriber("/realsense/frontCamera/color/image_raw",Image,self.image_callback)
		self.info_sub = rospy.Subscriber("/realsense/frontCamera/color/camera_info", CameraInfo, self.info_callback)

		# self.im_sub  = rospy.Subscriber("/frontCamera/color/image_raw",Image,self.image_callback)
		# self.info_sub = rospy.Subscriber("/frontCamera/color/camera_info", CameraInfo, self.info_callback)

		self.D = np.array([])
		self.K = np.array([])
		self.bridge = CvBridge()
		self.i = 0

	def image_callback(self,data):
			# self.i+=1
			# if self.i %20 ==0:
			# 	return
			img = self.bridge.imgmsg_to_cv2(data,"bgr8")

			h,w,_ = img.shape

			print(img.shape)

			
			
			
			cv2.imwrite('/home/kob51/Desktop/test_img.jpg',img)

			# points are (x,y) pairs
			# top left
			# top right
			# bottom left
			# bottom right

			src = np.float32([(200,300),
		          			  (450,300),
                  			  (0,410),
				  			  (639,410)])


			dst = np.float32([(100,0),
		          			  (w-100,0),
		          			  (100,h),
		          			  (w-100,h)])

			processed_img,birdseye = pipeline(img,self.K,self.D,src,dst)

			cv2.namedWindow("Bird's Eye View",cv2.WINDOW_NORMAL)
			cv2.imshow("Bird's Eye View",birdseye)
			cv2.namedWindow("Camera View",cv2.WINDOW_NORMAL)
			cv2.imshow("Camera View",processed_img)
			cv2.waitKey(10)


	def info_callback(self, data):
		if self.K.size == 0 and self.D.size == 0:
			K = data.K
			K = np.array(K)
			self.K = np.reshape(K,(3,3))
			D = data.D

			D = np.array(D)
			if D.size == 0:
				self.D = np.zeros(5)
			else:
				self.D = D
		else:
			print("**************************")
			self.info_sub.unregister()


if __name__ == "__main__":

	rospy.init_node('test')

	detector = LaneDetector()

	rospy.spin()