#!/usr/bin/env python

import sys
sys.dont_write_bytecode = True
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image as pImage

# ROS
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class LaneDetector:
    def __init__(self):

        self.bridge = CvBridge()
        self.im_sub  = rospy.Subscriber("/frontCamera/color/image_raw",Image,self.image_callback, queue_size=1)
        # other possible topics:
        # "/realsense/frontCamera/color/image_raw"
        # "/camera/color/image_raw"

        # define ROI
        self.ROI_UPPER_Y = 340
        self.ROI_RIGHT_X = 480

    def image_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")

        #=======ADDING CONTRAST OR BRIGHTNESS TESTING=====
        #beta = 30
        #img  = cv2.convertScaleAbs(img, alpha=1, beta=beta)
        #===================================================
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS).astype(np.float)

        # hls thresholding for yellow
        lower = np.array([15.0, 0.00*255, 0.90*255], dtype=np.uint8)
        upper = np.array([30.0, 1.00*255, 1.00*255], dtype=np.uint8)
        mask = cv2.inRange(hls, lower, upper)
        th = cv2.bitwise_and(rgb, rgb, mask=mask).astype(np.uint8)
        th = cv2.cvtColor(th, cv2.COLOR_HLS2RGB)
        th = cv2.cvtColor(th, cv2.COLOR_RGB2GRAY)
        th = cv2.GaussianBlur(th, (5, 5), 0)
        _, th = cv2.threshold(th, 60, 255, cv2.THRESH_BINARY)
	th = cv2.morphologyEx(th, cv2.MORPH_OPEN, (5,5))
	#th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, (5,5))

        # extract ROI
        mask = np.zeros_like(th)
        mask[self.ROI_UPPER_Y:, :self.ROI_RIGHT_X] = 1
        roi = cv2.bitwise_and(th, th, mask=mask)

        # detect center line
        edges = cv2.Canny(roi, 100, 200, apertureSize=3)
        minLineLength = 130
        maxLineGap = 20
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, np.array([]), minLineLength, maxLineGap)

        res = np.stack((th, th, th), axis=-1)

        # find the line w/ max length
        max_len, X1, Y1, X2, Y2 = 0, 0, 0, 0, 0
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                line_len = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                if max_len < line_len:
                    max_len = line_len
                    X1, Y1, X2, Y2 = x1, y1, x2, y2
        # draw line w/ max length or
        # track last detected line if no lines detected
        # draw previous result if tracking lost
        if max_len != 0:
            cv2.line(res, (X1, Y1), (X2, Y2), (0, 0, 255), 3)
            bb = (X1, Y1, np.abs(X2-X1), np.abs(Y2-Y1))
            self.tracker = cv2.TrackerKCF_create()
            self.tracker.init(roi, bb)
            self.lose_track_count = 0
        elif self.tracker is not None:
            success, bb = self.tracker.update(roi)
            x, y, w, h = [int(v) for v in bb]
            if success:
                cv2.line(res, (x, y), (x+w, y-h), (0, 0, 255), 3)
                self.lose_track_count = 0
            else:
                self.lose_track_count += 1
                # lost track for a short time only
                if self.lose_track_count < 10:
                    cv2.line(res, (X1, Y1), (X2, Y2), (0, 0, 255), 3)


        """
        # draw all lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(res, (x1, y1), (x2, y2), (0, 0, 255), 3)
        """

        # draw rectangle on ROI
        h, w, _ = img.shape
        self.ROI_UPPER_LEFT = (0, self.ROI_UPPER_Y)
        self.ROI_LOWER_RIGHT = (self.ROI_RIGHT_X, h)
        color = (255, 0, 0) # blue in BGR
        thickness = 2
        res = cv2.rectangle(res, self.ROI_UPPER_LEFT, self.ROI_LOWER_RIGHT, color, thickness)

        # concatenate data & result
        concat = np.concatenate((img, res), axis=1)
        cv2.imshow("Lane Detection", concat)
        cv2.waitKey(10)

if __name__ == "__main__":
    rospy.init_node('test')
    detector = LaneDetector()
    rospy.spin()
