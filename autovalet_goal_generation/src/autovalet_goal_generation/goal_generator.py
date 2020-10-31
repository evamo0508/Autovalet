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
from tf.transformations import quaternion_from_euler

class goal_generator:
    def __init__(self, goal_frame, dist_from_line_start=2, goal_dist_factor=1.2):
        '''
        goal_frame      : [string] specifies the name of the frame in which goal is required to be published
        '''

        # Frames in which goals are to be generated
        self.goal_frame_id  = goal_frame

        # control variable for distance from closest center line point
        self.dist_from_line_start = dist_from_line_start
        # control variable for choosing target point based on line length [must be less than 1]
        self.goal_dist_factor     = goal_dist_factor

        # tf stuff for transforming to global goal frame
        self.tf_buffer      = tf2_ros.Buffer(rospy.Duration(1200.0)) # Length of tf2 buffer (?)
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)
        self.costmap_height = rospy.get_param('/move_base/global_costmap/height')

    def generate_goal_from_egoline(self, ego_line, egoline_frame):
        '''
        Input:
        egolane         : [nx3 np.ndarray] contains the position of the 3D points of the lane
        egoline_frame   : [string] specifies the name of the frame in which the egoline pointcloud is defined

        Output:
        goal_pose       : [PoseStamped] contains the goal pose in the goal_frame
        '''

        # ref: https://stackoverflow.com/questions/2298390/fitting-a-line-in-3d/2333251#2333251
        ego_line         = ego_line[ego_line[:, 2].argsort()]
        egoline_midpoint = ego_line.mean(axis=0)
        _, _, Vt         = np.linalg.svd(ego_line - egoline_midpoint)
        line_direction   = Vt[0]    # The principal direction of the distribution. Line direction here

        # Correct sign to ensure that the direction is always away from the robot
        # The fix is to ensure the z-direction is always positive which means direction away from the robot
        line_direction = np.sign(line_direction[-1]) * line_direction

        '''
        Different distances to generate the goal
        (depending on whether the midpoint is in the costmap)
        1. the 25% close point in the line cloud
        2. retract fixed fraction of length between midpoint & costmap edge
           to bring the point back in the costmap but still far
        '''
        # generate target pos
        if egoline_midpoint[2] < 0.5 * self.costmap_height:
            target_point = ego_line[int(ego_line.shape[0]/3)]
        else:
            dist_to_costmap  = egoline_midpoint[2] - 0.5 * self.costmap_height
            target_point = egoline_midpoint - self.goal_dist_factor * line_direction * dist_to_costmap

        # Convert the direction vector to quaternion
        line_quaternion = direction_vector_to_quaternion(line_direction)

        # Populate a pose message
        goal_cam_frame = make_pose_stamped(egoline_frame, target_point, line_quaternion)

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

    def generate_goal_for_left_turn(self, husky_frame):
        '''
        Input:
        husky_frame   : [string] specifies the name of the frame in which Husky is in

        Output:
        goal_pose       : [PoseStamped] contains the goal pose in the goal_frame
        '''
        target_pos = np.array([3.0, 1.5, 0.0])
        target_quaternion = quaternion_from_euler(0, 0, np.pi/8)

        # Populate a pose message
        goal_husky_frame = make_pose_stamped(husky_frame, target_pos, target_quaternion)

        # Transform the pose to the goal frame ID
        transform = self.tf_buffer.lookup_transform(self.goal_frame_id, # target_frame_id
                                   husky_frame, #source frame
                                   rospy.Time(0), #get the tf at first available time
                                   rospy.Duration(1.0)) #timeout after 1

        # Transform goal from camera frame to map frame
        goal = do_transform_pose(goal_husky_frame, transform)

        # Force 2D goal constraints [without which planner would complain -_-]
        goal_2D = force2DGoal(goal)
        return goal_2D
