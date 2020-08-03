#!/usr/bin/env python

import sys
import os

#ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

#if ros_path in sys.path:
#    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy
import matplotlib.pyplot as plt

from erfnet import ERFNet
from erfnet_imagenet import ERFNet as ERFNet_imagenet
from transform import Relabel, ToLabel, Colorize
import torch
from torch.autograd import Variable
from PIL import Image as pImage
from torchvision.transforms import ToTensor, ToPILImage
from torchvision.transforms import Compose,Normalize,Resize

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class LaneDetector:
    def __init__(self):
        self.model = None
        self.im_sub  = rospy.Subscriber("/realsense/frontCamera/color/image_raw",Image,self.image_callback, queue_size=1)
        self.info_sub = rospy.Subscriber("/realsense/frontCamera/color/camera_info", CameraInfo, self.info_callback)

        # do seg every 10 frames
        self.count = 0

        # self.im_sub  = rospy.Subscriber("/frontCamera/color/image_raw",Image,self.image_callback)
        # self.info_sub = rospy.Subscriber("/frontCamera/color/camera_info", CameraInfo, self.info_callback)

        self.D = np.array([])
        self.K = np.array([])
        self.bridge = CvBridge()

        #self.weightspath = "../include/lane_detection/erfnet_pretrained.pth"
        self.weightspath = "/home/evamo0508/catkin_ws/src/autovalet/lane_detection/include/lane_detection/model_best3.pth"

        self.num_channels = 3
        self.num_classes= 1
        self.image_transform = Compose([
            Resize((512,1024),pImage.BILINEAR),
            ToTensor(),
            #Normalize([.485, .456, .406], [.229, .224, .225]),
        ])
        self.target_transform = Compose([
            # Resize((512,1024),Image.NEAREST),
            Relabel(1, 255),
            ToPILImage(),
            Resize((480,640), pImage.NEAREST),
        ])

        erfnet_imagenet = ERFNet_imagenet(1000)
        pretrainedEnc = torch.nn.DataParallel(erfnet_imagenet)
        pretrainedEnc = next(pretrainedEnc.children()).features.encoder
        pretrainedEnc = pretrainedEnc.cpu()
        self.model = ERFNet(self.num_classes, encoder=pretrainedEnc)
        self.model = torch.nn.DataParallel(self.model)
        self.model = self.load_my_state_dict(self.model,torch.load(self.weightspath, map_location=torch.device('cpu')))
        self.model = self.model.eval()
        

    def load_my_state_dict(self, model, state_dict):  #custom function to load model when not all dict elements
        own_state = model.state_dict()
        for name, param in state_dict.items():
            if name not in own_state: 
                continue
            own_state[name].copy_(param)
    
        return model

    def image_callback(self,data):
        if self.model is None:
            return
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")

        if self.count % 10 == 0:
            #=======ADDING CONTRAST OR BRIGHTNESS TESTING=====
            alpha = np.random.random_sample() + 0.5       
            beta = np.random.randint(0, 80) - 40
            img  = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
            #=======ADDING GAUSSIAN NOISE=====================
            #sigma = 3
            #gauss = np.random.normal(0, sigma, img.shape).reshape(img.shape)
            #img = (img + gauss).astype(np.uint8)
            #===================================================
            self.img = img
            pImg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pImg = pImage.fromarray(pImg.astype(np.uint8)).convert('RGB')

            inputs = self.image_transform(pImg)
            inputs = Variable(inputs)
            inputs = inputs.unsqueeze(0)
            with torch.no_grad():
                outputs = self.model(inputs)

            pred = torch.where(outputs > 0.1, torch.ones([1], dtype=torch.uint8), torch.zeros([1], dtype=torch.uint8))
            pred = pred.squeeze(0)
            pred = self.target_transform(pred)
            
            # concatenate data & result
            pred = np.asarray(pred, dtype=np.uint8) # only contains 0, 255 at this point
            proc = pred.copy()
            proc[:proc.shape[0]/2] = 0
            #se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (40,40))
            se1 = np.ones((10,10), np.uint8)
            proc = cv2.dilate(proc, se1, 1)
            #se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (60,60))
            se2 = np.ones((30,30), np.uint8)
            #mask = cv2.morphologyEx(proc, cv2.MORPH_CLOSE, se1)
            mask = cv2.morphologyEx(proc, cv2.MORPH_OPEN, se2)
            proc = proc * (mask / 255)
            proc = np.stack((proc, proc, proc), axis=-1)
            self.proc = proc
            pred = np.stack((pred, pred, pred), axis=-1)
            self.pred = pred
        
        
        concat = np.concatenate((self.proc, self.pred), axis=1)
        #concat = np.concatenate((self.img, self.pred), axis=1)
        cv2.imshow("Lane Detection", concat)
        cv2.waitKey(10)
        self.count += 1

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
