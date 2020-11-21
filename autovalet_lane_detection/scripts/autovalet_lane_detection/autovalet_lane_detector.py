#!/usr/bin/env python

import sys
sys.dont_write_bytecode = True
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image as pImage
import tf2_ros
import time

# ROS
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf.transformations import quaternion_from_matrix, rotation_matrix, euler_from_quaternion, quaternion_from_euler

# Goal generation stuff
from autovalet_goal_generation.goal_generator import goal_generator
import autovalet_goal_generation.utils

# Lane detection stuff
from autovalet_lane_detection.LaneDetector import LaneDetector

class lane_detection_node:

    def __init__(self, color_topic, depth_topic, colorInfo_topic, laneCloud_topic, egoLine_topic, hlsBounds, lineParams):
        self.lane_det_initialized = False
        self.bridge    = CvBridge()

        # Setup subscribers
        self.color_sub = Subscriber(color_topic, Image)
        self.depth_sub = Subscriber(depth_topic, Image)

        # A handle for publishing goal poses
        self.goal_handle   = rospy.Publisher("move_base_simple/goal_fake", PoseStamped, queue_size=1)
        self.av_goal_generator = goal_generator("map")

        # Looking to synchronize both the topics within a 1/10th of a second
        self.ats = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=2, slop=0.5)
        self.ats.registerCallback(self.registered_image_callback)

        # time tracker for publishing goals at a lower rate
        self.previous_time = rospy.get_time()

        # Create a lane detector object
        self.lane_detector = LaneDetector(colorInfo_topic, laneCloud_topic, egoLine_topic, hlsBounds, lineParams, sim=False, debug=False)

        self.lane_det_initialized = True

    def registered_image_callback(self, color_msg, depth_msg):
        # cvBridge image
        color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # lane detection algo
        if self.lane_det_initialized:
            _, ego_line, _ = self.lane_detector.detectLaneRGBD(color_img, depth_img)


        # generate goal from the egoline
        if((rospy.get_time() - self.previous_time) > 2 and ego_line is not None):
            goal_pose = self.av_goal_generator.generate_goal_from_egoline(ego_line, depth_msg.header.frame_id)
            self.goal_handle.publish(goal_pose)
            self.previous_time = rospy.get_time()

if __name__ == "__main__":
    rospy.init_node('lane_detector_node')

    color_topic     = "/frontCamera/color/image_raw"
    depth_topic     = "/depth_registered/image_rect"
    colorInfo_topic = "/frontCamera/color/camera_info"
    laneCloud_topic = "/lane/pointCloud"
    egoLine_topic   = "/lane/egoLine"

    # Load Params
    hlsBounds = rospy.get_param('/lane_detection/hlsbounds')
    lineParams = rospy.get_param('/lane_detection/lineparams')

    detector = lane_detection_node(color_topic, depth_topic, colorInfo_topic, laneCloud_topic, egoLine_topic, hlsBounds, lineParams)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
