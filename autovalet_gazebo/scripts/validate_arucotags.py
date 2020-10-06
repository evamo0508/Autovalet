#!/usr/bin/env python
# license removed for brevity

# Author: Nikita Mishra
# Adapted from: validate_localization.py
# Date of creation: September 25, 2020 04:20 PM
# Description: Validation script for Progress Review 8, compare arucotags pose (estimate) with gt.pose


import rospy
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose

import numpy as np
import matplotlib.pyplot as plt
import os

class aruco_tags_validation:
    def __init__(self, gt_topic, est_topic): 
        self.gt_handle = rospy.Subscriber(gt_topic, LinkStates, self.gt_cb)
        self.est_handle = rospy.Subscriber(est_topic, PoseStamped, self.est_cb) ## Odometry --> PoseStamped
        
        self.gt = None
        self.estimate = None
        
        """
        To ping the first message of gt and populate the world to pose offset
        """
        # self.isFirstCb = True
        self.offset = None
        self.positionErrorLog = []
        self.yawErrorLog = []
        self.runningAverage = []

    def gt_cb(self, gt):
        """
        localization validation callback function
        """

        #TODO: get list index from name of first message
        self.gt = gt.pose[-7] ## 7th (from last) name 'aruco_visual_marker_7::marker'
        self.offs = gt.pose[-5] #ground truth published by gazebo link state /::base_link w.r.t the gazebo world origin
        self.offset = np.array([self.offs.position.x,  self.offs.position.y, self.offs.position.z, self.quat_to_euler(self.offs.orientation)[2]])

    def est_cb(self, est):

        self.estimate = est.pose #pose.pose in nav_msgs --> .pose in geometry_msgs

        if(self.gt is not None):
            cur_pose = np.array([self.estimate.position.x, self.estimate.position.y,self.estimate.position.z, self.quat_to_euler(self.estimate.orientation)[2]])
            gt_pose = np.array([self.gt.position.x, self.gt.position.y,self.gt.position.z, self.quat_to_euler(self.gt.orientation)[2]])
            print"G.T. pose: ", (gt_pose - self.offset)[:-1]
            print"Current pose estimate: ", (cur_pose)[:-1]
            print"Individual Errors (in cms: x, y, z) ", (cur_pose - (gt_pose - self.offset))[:-1]*100
            print"==============================================="
                
    def quat_to_euler(self, q):
        # euler indexing as RPY
        quaternion = (q.x, q.y, q.z, q.w)
        euler = euler_from_quaternion(quaternion)
        return euler


def main():
    np.set_printoptions(suppress=True, precision=4)

    print("Starting Aruco Tags Detection Error calculator node")
    rospy.init_node("aruco_tags_validation", anonymous=False)
    
    ground_truth_topic = '/gazebo/link_states'
    estimate_topic = '/aruco_single/pose'
    noise = 0.008

    errorHandler = aruco_tags_validation(ground_truth_topic, estimate_topic)

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    main()
