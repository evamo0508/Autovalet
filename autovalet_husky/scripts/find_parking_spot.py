#!/usr/bin/env python
'''
Script to generate parking goal pose from detecting an april tag

Date    : 6 Oct 2020

Changelog:
    subbu - refactoring to work with tag topic than TF
'''
import numpy as np

# Include ROS libs
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Include messages
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix

class parking_spot:
    '''
    Module for defining parking spot parameters
    '''
    def __init__(self, goal_topic, tag_topic, goal_frame_id, aruco_frame_name):
        # Setup listeners and talkers
        self.pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(tag_topic, PoseStamped, self.generate_parking_goal)

        self.tf_buffer      = tf2_ros.Buffer(rospy.Duration(1200.0)) # Length of tf2 buffer (?)
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)
        self.aruco_frame_name = aruco_frame_name
        self.goal_frame_id  = goal_frame_id

        # Make the origin of the tag used for looking up later!
        self.tag_origin = PoseStamped()
        self.tag_origin.pose.position.x = 0
        self.tag_origin.pose.position.y = 0
        self.tag_origin.pose.position.z = 4

        x,y,z,w = quaternion_from_euler(0, -np.pi/2, -np.pi/2)
        self.tag_origin.pose.orientation.x = x
        self.tag_origin.pose.orientation.y = y
        self.tag_origin.pose.orientation.z = z
        self.tag_origin.pose.orientation.w = w

    def generate_parking_goal(self, tag_pose):
        '''
        Final parking pose published to movebase
        '''
        self.aruco_frame_id = tag_pose.header.frame_id

        transform = self.tf_buffer.lookup_transform(self.goal_frame_id, # target_frame_id
                                   self.aruco_frame_name, #source frame
                                   rospy.Time(0), #get the tf at first available time
                                   rospy.Duration(1.0)) #timeout after 1


        # Get the goal in correct frame
        goal = do_transform_pose(self.tag_origin, transform)
        goal.pose.position.z = 0                    # Enforce 2D nav constraint
        # Correct the orientation of the tag (enforce roll and pitch zero)
        goal.pose.orientation = self.orientation_correction(goal.pose.orientation)

        # Let's go move base!
        self.pub.publish(goal)

    def orientation_correction(self, quat):
        '''
        Returns a quaternion after forcing the roll and pitch to zero
        '''
        roll, pitch, yaw = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        x, y, z, w = quaternion_from_euler(0, 0, yaw)

        corrected_quat = Quaternion()
        corrected_quat.x = x
        corrected_quat.y = y
        corrected_quat.z = z
        corrected_quat.w = w

        return corrected_quat

def main():
    rospy.init_node('parking_spot_node')

    # goal topic is the name of the topic movebase/any planner expects its goals from
    # tag_topic is the name of the topic in which the pose of the marker is being published
    # goal_pose_frame_id is the name of the frame in which you want the goal to be generated
    # aruco_frame_name is the NAME OF THE FRAME PUBLISHED AS THE ARUCO TAG
    tag_topic = '/aruco_single/pose'
    goal_topic = '/move_base_simple/goal'
    goal_pose_frame_id = 'map'
    aruco_frame_name = 'parking_spot' #'aruco_marker_frame'
# 'parking_spot' # (Needs to be same as what is set in aruco launcher)

    av_spot = parking_spot(goal_topic, tag_topic, goal_pose_frame_id, aruco_frame_name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
