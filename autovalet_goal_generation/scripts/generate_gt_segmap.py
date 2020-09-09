#!/usr/bin/env python
"""
Script publishes a segmentation map from the artificially colored
ground plane for transferring the lane constraint to costmap

Author  : Subramanian Krishnan (subramak@andrew.cmu.edu)
Date    : 07 Sep 2020

Changelog:
    subbu - 9/07 - Initial commit
"""

import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class ground_truth:
    '''
    Wrapper class to process the "artificial" ground plane
    '''
    def __init__(self, output_topic, input_topic):
        self.pub = rospy.Publisher(output_topic, Image, queue_size=1)
        self.sub = rospy.Subscriber(input_topic, Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, rosImage):
        # process image
        img = self.bridge.imgmsg_to_cv2(rosImage,"bgr8")

        # Convert BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        # Range found from the hsv configuration tool in utils
        low = np.array([48,156,222])
        high = np.array([150,255,255])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, low, high)
        kernel = np.ones((7,7),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        try:
            ros_segmap_image = self.bridge.cv2_to_imgmsg(mask, "mono8")
            ros_segmap_image.header.stamp = rospy.Time.now()
            ros_segmap_image.header.frame_id = rosImage.header.frame_id
            self.pub.publish(ros_segmap_image)
        except CvBridgeError as e:
            print(e)
def main():
  rospy.init_node('lane_ground_truth', anonymous=False)

  gt_topic = "/lane/ground_truth"
  input_topic = "/frontCamera/color/image_raw"

  gt = ground_truth(gt_topic, input_topic)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
