#!/usr/bin/env python

import sys
sys.dont_write_bytecode = True
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image as pImage

# pytorch
import torch
from torch.autograd import Variable
from torchvision.transforms import ToTensor, ToPILImage
from torchvision.transforms import Compose,Normalize,Resize

# ROS
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# ERFNet
from erfnet import ERFNet
from erfnet_imagenet import ERFNet as ERFNet_imagenet
from transform import Relabel, ToLabel, Colorize

class LaneDetector:
    def __init__(self):
        # do seg every 10 frames
        self.count = 0

        self.model = None
        self.weightspath = "/home/evamo0508/catkin_autovalet/src/autovalet/autovalet_lane_detection/include/lane_detection/erfnet_pretrained.pth"
        self.num_channels = 3
        self.num_classes = 20

        # img transforms
        self.image_transform = Compose([
            Resize((512, 1024), pImage.BILINEAR),
            ToTensor(),
        ])
        self.pred_transform = Compose([
            ToPILImage(),
            Resize((480,640), pImage.NEAREST),
        ])

        # load model
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = ERFNet(self.num_classes)
        self.model = torch.nn.DataParallel(self.model)
        self.model = self.load_my_state_dict(self.model,torch.load(self.weightspath, map_location=self.device))
        self.model = self.model.eval()
        self.model.to(self.device)

        self.bridge = CvBridge()
        self.im_sub  = rospy.Subscriber("/camera/color/image_raw",Image,self.image_callback, queue_size=1)
        # other possible topics:
        # "/realsense/frontCamera/color/image_raw"
        # "/frontCamera/color/image_raw"

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
            beta = 20
            img  = cv2.convertScaleAbs(img, alpha=1, beta=beta)
            #===================================================
            self.img = img
            pImg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pImg = pImage.fromarray(pImg.astype(np.uint8)).convert('RGB')

            inputs = self.image_transform(pImg)
            inputs = Variable(inputs).to(self.device)
            inputs = inputs.unsqueeze(0) # since this is a single img
            with torch.no_grad():
                outputs = self.model(inputs)

            pred = outputs[0].max(0)[1].byte().cpu().data
            pred = Colorize()(pred.unsqueeze(0))
            pred = self.pred_transform(pred)

            # concatenate data & result
            pred = np.asarray(pred, dtype=np.uint8)
            self.pred = pred

        concat = np.concatenate((self.img, self.pred), axis=1)
        cv2.imshow("Lane Detection", concat)
        cv2.waitKey(10)
        self.count += 1

if __name__ == "__main__":
    rospy.init_node('test')
    detector = LaneDetector()
    rospy.spin()
