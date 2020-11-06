#!/usr/bin/env python
'''
Script to generate parking goal pose from detecting an april tag

Date    : 6 Oct 2020

Changelog:
    subbu - refactoring to work with tag topic than TF
    eva - generate 2 waypoints
'''
import numpy as np

# Include ROS libs
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Include messages
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix

class parking_spot:
    '''
    Module for defining parking spot parameters
    '''
    def __init__(self, goal_topic, tag_topic, goal_frame_id, husky_frame_id, aruco_frame_name,debug=True):
        # for accumulating tag poses
        self.count = 0
        self.tag_buffer_size = 20
        self.tfArray = []

        # Setup listeners and talkers
        self.pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(tag_topic, PoseStamped, self.collect_tag_poses)

        self.tf_buffer        = tf2_ros.Buffer(rospy.Duration(1200.0)) # Length of tf2 buffer (?)
        self.tf_listener      = tf2_ros.TransformListener(self.tf_buffer)
        self.aruco_frame_name = aruco_frame_name
        self.goal_frame_id    = goal_frame_id
        self.husky_frame_id   = husky_frame_id
        self.first_goal_close = False
        self.debug = debug
        self.line = None
        self.linecenter = None

        # helper const & bool
        self.costmap_height              = rospy.get_param('/move_base/global_costmap/height')
        self.first_goal_in_costmap       = False
        self.first_goal_is_close_meter   = 0.8
        self.debug = debug

    def collect_tag_poses(self, tag_pose):
        '''
        Take in numOfTags tag poses and do RANSAC in the end to avoid outliers
        '''
        if self.count < self.tag_buffer_size: # collect tag poses
            self.aruco_frame_id = tag_pose.header.frame_id

            tf = self.tf_buffer.lookup_transform(self.goal_frame_id, # target_frame_id
                                    self.aruco_frame_name, # source frame
                                    rospy.Time(0), # get the tf at first available time
                                    rospy.Duration(1.0)) # timeout after 1
            self.tfArray.append(tf)
            self.count += 1
        elif self.count == self.tag_buffer_size: # perform RANSAC on transformArray
            self.tag_tf = self.tag_pose_RANSAC()
            self.pub_two_goals()
            self.count += 1
            self.goal1 = None
            self.goal2 = None

    def pub_two_goals(self):

        self.linecenter = self.line # to stop recalc of centerline midpt
        if self.linecenter.y > self.tag_tf.transform.translation.y:
            rhs = True
            pos1 = [0, 3, 1]
        else:
            rhs = False
            pos1 = [0, 1.5, -1]
            # rot1 = [0 , np.pi/4, -np.pi/4]
            print("line center y value : ", self.linecenter.y)
            print("tag y : ", self.tag_tf.transform.translation.y)
        # current params for right turns only
        # pos1 = [0, 3, 1]
        rot1 = [0 , np.pi/4, -np.pi/4]
        self.goal1 = self.generate_parking_goal(self.tag_tf, pos1, rot1)
        
        # keep moving w/ goal gen until goal1 is in costmap
        while self.dist_to_goal(self.goal1) > 0.48 * self.costmap_height:
            # rospy.sleep(0.1)
            a = 1 #continue
        self.first_goal_in_costmap = True

        # keep publishing 1st goal until the robot is close enough
        while self.dist_to_goal(self.goal1) > self.first_goal_is_close_meter:
            rospy.sleep(0.5)
            self.pub.publish(self.goal1)
            if self.debug:
                print("first goal published")

        # pub 2nd goal once
        if rhs: #self.linecenter.y > self.goal1.pose.position.y:
            pos2 = [0, 0, 4]
            rot2 = [0, -np.pi/2, -np.pi/2]
            if self.debug:
                print"**********right spot*********"
                print ("tfs are: ", self.linecenter.y, self.goal1.pose.position.y)
        else:
            pos2 = [0, 0, -4]
            rot2 = [0, np.pi/2, np.pi/2]
            if self.debug:
                print"***********left spot********"
                print ("tfs are: ", self.linecenter.y, self.goal1.pose.position.y)
        # pos2 = [0, 0, 4]
        # rot2 = [0, -np.pi/2, -np.pi/2]
        self.goal2 = self.generate_parking_goal(self.tag_tf, pos2, rot2)
        self.pub.publish(self.goal2)
        if self.debug:
            print("second goal published")        
        return

    def dist_to_goal(self, goal):
        base_link_tf = self.tf_buffer.lookup_transform(self.goal_frame_id, # target_frame_id
                                self.husky_frame_id, #source frame
                                rospy.Time(0), #get the tf at first available time
                                rospy.Duration(1.0)) #timeout after 1
        base_link_x = base_link_tf.transform.translation.x
        base_link_y = base_link_tf.transform.translation.y
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        dist = np.linalg.norm(np.array([base_link_x - goal_x, base_link_y - goal_y]))

        return dist

    def tag_pose_RANSAC(self):
        '''
        distance between quaternions is calculated based on the formula here:
        https://math.stackexchange.com/questions/90081/quaternion-distance
        '''
        quatArray = []
        for i in range(len(self.tfArray)):
            quat = self.tfArray[i].transform.rotation
            quatArray.append(np.array([quat.x, quat.y, quat.z, quat.w]))
        votes = []
        for i in range(len(quatArray)):
            vote = 0
            for j in range(len(quatArray)):
                diff = np.abs(1 - np.dot(quatArray[i], quatArray[j]) ** 2)
                vote = vote + 1 if diff < 0.005 else vote
            votes.append(vote)
        # pick the pose with the most inliers
        tf = self.tfArray[votes.index(max(votes))]

        return tf

    def generate_parking_goal(self, tf, pos, rot):
        '''
        waypoint published to movebase
        '''
        tag2waypoint = PoseStamped()
        tag2waypoint.header.frame_id = 'parking_spot'
        tag2waypoint.pose.position.x = pos[0]
        tag2waypoint.pose.position.y = pos[1]
        tag2waypoint.pose.position.z = pos[2]

        x,y,z,w = quaternion_from_euler(rot[0], rot[1], rot[2])
        tag2waypoint.pose.orientation.x = x
        tag2waypoint.pose.orientation.y = y
        tag2waypoint.pose.orientation.z = z
        tag2waypoint.pose.orientation.w = w

        # Get the goal in correct frame
        goal = do_transform_pose(tag2waypoint, tf)
        goal.pose.position.z = 0 # Enforce 2D nav constraint
        # Correct the orientation of the tag (enforce roll and pitch zero)
        goal.pose.orientation = self.orientation_correction(goal.pose.orientation)

        return goal

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
    tag_topic          = '/ARUCO/pose'
    goal_topic         = '/move_base_simple/goal'
    goal_pose_frame_id = 'map'
    husky_frame_id     = 'base_link'
    aruco_frame_name   = 'parking_spot' #'aruco_marker_frame' or 'parking_spot'
                         # (Needs to be same as what is set in aruco launcher)

    av_spot = parking_spot(goal_topic, tag_topic, goal_pose_frame_id, husky_frame_id, aruco_frame_name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
