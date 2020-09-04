#!/usr/bin/env python


import sys
import os



ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy

from erfnet import ERFNet
import torch
from PIL import Image
from torchvision.transforms import ToTensor, ToPILImage
from torchvision.transforms import Compose,CenterCrop,Normalize,Resize



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

		self.weightspath = "../include/lane_detection/erfnet_pretrained.pth"

		self.num_channels = 3
		self.num_classes=20
		self.image_transform = Compose([
    										Resize((512,1024),Image.BILINEAR),
    										ToTensor(),
    										#Normalize([.485, .456, .406], [.229, .224, .225]),
											])
		self.target_transform = Compose([
    										# Resize((512,1024),Image.NEAREST),
    									Resize((480,640),Image.NEAREST),
										])

		self.model = ERFNet(self.num_classes)
		self.model = torch.nn.DataParallel(self.model)
		self.model = self.load_my_state_dict(self.model,torch.load(self.weightspath))
		self.model = self.model.eval()

		def load_my_state_dict(self, model, state_dict):  #custom function to load model when not all dict elements
			own_state = model.state_dict()
			for name, param in state_dict.items():
			    if name not in own_state: continue
			    own_state[name].copy_(param)

		return model

	def image_callback(self,data):
			self.i+=1
			if self.i %2 ==0:
				return
			img = self.bridge.imgmsg_to_cv2(data,"bgr8")
			# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
			# ROS_ASSERT(cv2.imwrite('~/Desktop/test_img.jpeg',img))
			# cv2.imwrite('/home/kob51/Desktop/test_img'+str(self.i)+'.jpg',img)
			# ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) )
			# img = np.array(img)
            
			# foo = img.copy()

			# img = self.image_transform(img)
			# img = Variable(img)
			# out = self.model(img)

			# label = outputs[0].max(0)[1].byte().cpu().data
			# label_color = Colorize()(label.unsqueeze(0))

			# label_save = ToPILImage()(label_color)
			# label_save = target_transform(label_save)

			# label_save = np.array(label_save.convert('RGB'))
			# label_save = label_save[:,:,::-1].copy()

			# cv2.namedWindow("Segmentation",cv2.WINDOW_NORMAL)
			# cv2.imshow("Bird's Eye View",label_save)




			h,w,_ = img.shape

            
			print(img.shape)

			src = np.float32([(676,590),
							(1180,590),
							(0,800),
		                  (1919,800)])


			dst = np.float32([(600,0),
		                  (w-600,0),
		                  (600,h),
		                  (w-600,h)])

			processed_img,birdseye = pipeline(img,self.K,self.D,src,dst)

			processed_img,birdseye = img, img

			img = cv2.polylines(img,[src],True,(0,255,255))


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
