'''
Main class for the goal generator

Author  : subramak@andrew.cmu.edu
Date    : 19 Oct 2020

Changelog:
    subbu - 10/19 - refactor goal generator
'''

# custom modules
from utils import force2DGoal, direction_vector_to_quaternion, make_pose_stamped

# python modules
import numpy as np

# ROS modules
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class goal_generator:
    def __init__(self, goal_frame):
        '''
        goal_frame      : [string] specifies the name of the frame in which goal is required to be published
        '''

        # Frames in which goals are to be generated
        self.goal_frame_id  = goal_frame

        # tf stuff for transforming to global goal frame
        self.tf_buffer      = tf2_ros.Buffer(rospy.Duration(1200.0)) # Length of tf2 buffer (?)
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)
    
    def generate_goal_from_egoline(self, ego_line, egoline_frame):
        '''
        Input:
        egolane         : [nx3 np.ndarray] contains the position of the 3D points of the lane
        egoline_frame   : [string] specifies the name of the frame in which the egoline pointcloud is defined

        Output:
        goal_pose       : [PoseStamped] contains the goal pose in the goal_frame
        '''

        # ref: https://stackoverflow.com/questions/2298390/fitting-a-line-in-3d/2333251#2333251
        egoline_midpoint = ego_line.mean(axis=0)
        _, _, Vt         = np.linalg.svd(ego_line - egoline_midpoint)
        line_direction   = Vt[0]    # The principal direction of the distribution. Line direction here
        
        # Correct sign to ensure that the direction is always away from the robot
        # The fix is to ensure the z-direction is always positive which means direction away from the robot
        line_direction = np.sign(line_direction[-1])*line_direction

        # Convert the direction vector to quaternion
        line_quaternion = direction_vector_to_quaternion(line_direction)

        # Populate a pose message
        goal_cam_frame = make_pose_stamped(egoline_frame, egoline_midpoint, line_quaternion)

        # Transform the pose to the goal frame ID
        transform = self.tf_buffer.lookup_transform(self.goal_frame_id, # target_frame_id
                                   egoline_frame, #source frame
                                   rospy.Time(0), #get the tf at first available time
                                   rospy.Duration(1.0)) #timeout after 1
    
        # Transform goal from camera frame to map frame
        goal = do_transform_pose(goal_cam_frame, transform)
        
        # Force 2D goal constraints [without which planner would complain -_-]
        goal_2D = force2DGoal(goal)

        return goal_2D
