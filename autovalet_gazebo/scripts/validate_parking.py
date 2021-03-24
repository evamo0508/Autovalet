#!/usr/bin/env python
# license removed for brevity

# Author: Eva Mo
# Adapted from: validate_arucotags.py
# Date of creation: October 9, 2020 04:20 PM
# Description: Validation script for Progress Review 9, compare husky final pose with arucotags gt.pose + 4m into the right spot

import os
import numpy as np

import rospy
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion

class parking_pose_validation:
    def __init__(self, gt_topic):
        # gt: ground truth published by gazebo link state
        self.gt = rospy.wait_for_message(gt_topic, LinkStates)
        self.aruco_tag_gt = None
        self.base_link_gt = None

        self.target_pose = self.get_target_pose()
        self.reached_pose = self.get_reached_pose()
        self.calculate_error()

    def get_target_pose(self):
        self.aruco_tag_gt    = self.gt.pose[-7] # name 'aruco_visual_marker_7::marker'
        # x += 4 compensating for the translation between parking spot & tag
        aruco_tag_gt_2d = np.array([self.aruco_tag_gt.position.x , self.aruco_tag_gt.position.y - 4, self.quat_to_euler(self.aruco_tag_gt.orientation)[2]])

        return aruco_tag_gt_2d

    def get_reached_pose(self):
        self.base_link_gt    = self.gt.pose[-5] # base_link w.r.t the gazebo world origin
        base_link_gt_2d = np.array([self.base_link_gt.position.x, self.base_link_gt.position.y, self.quat_to_euler(self.base_link_gt.orientation)[2]])

        return base_link_gt_2d

    def calculate_error(self):
        # Error in x and y in cms
        trans_err = (self.target_pose[:-1] - self.reached_pose[:-1])*100
        # Error in orientation (yaw) in degrees
        # make sure the orientation is the same in hardware so that the "270" works as well
        yaw_err   = np.abs((self.target_pose[2]*180/3.14 - 270) - self.reached_pose[2]*180/3.14) #comparing the aruco tag (rotated by 270deg) with husky's yaw
        print "Target pose (x,y,theta): ", self.target_pose[:-1]*100, self.target_pose[2]*180/3.14 - 270
        print "Reached pose (x,y,theta): ", self.reached_pose[:-1]*100, self.reached_pose[2]*180/3.14
        print "Translation error (cms): ", trans_err
        print "Orientation error (deg): ", yaw_err

    def quat_to_euler(self, q):
        # euler indexing as RPY
        quaternion = (q.x, q.y, q.z, q.w)
        euler = euler_from_quaternion(quaternion)
        return euler

def main():
    np.set_printoptions(suppress=True, precision=4)

    print("Starting Parking Pose Error calculator node")
    rospy.init_node("parking_pose_validation", anonymous=False)

    ground_truth_topic = '/gazebo/link_states'

    errorHandler = parking_pose_validation(ground_truth_topic)

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    main()
