'''
Contains utilities for goal generation. These helpers are reusable!

Author  : subramak@andrew.cmu.edu
Date    : 19 Oct 2020

Changelog:
    subbu - 10/19 - initial commit
'''

# python modules
import numpy as np

# ROS methods
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

def force2DGoal(pose):
        '''
        Input:
        pose            : [PoseStamped] contains goal on which 2D constraints need to be enforced.

        Output:
        goal_pose_2D    : [PoseStamped] contains the output goal pose which has z=0 and pitch, roll = 0
        '''

        goal_pose_2D = PoseStamped()

        # repopulate unchanged contents for goal_pose_2D
        # [doing it this way because I'm not sure if there's a copy constructor for PoseStamped messages]
        goal_pose_2D.header = pose.header
        goal_pose_2D.pose.position.x = pose.pose.position.x
        goal_pose_2D.pose.position.y = pose.pose.position.y

        # Perform corrections to make 2D nav goal
        goal_pose_2D.pose.position.z = 0        # Force pose to ground

        # Force pitch and roll to zero
        _, _, yaw = euler_from_quaternion((pose.pose.orientation.x,
                                                  pose.pose.orientation.y,
                                                  pose.pose.orientation.z,
                                                  pose.pose.orientation.w))
        x, y, z, w = quaternion_from_euler(0, 0, yaw)

        goal_pose_2D.pose.orientation.x = x
        goal_pose_2D.pose.orientation.y = y
        goal_pose_2D.pose.orientation.z = z
        goal_pose_2D.pose.orientation.w = w

        return goal_pose_2D

def direction_vector_to_quaternion(direction_vector, reference_vector = np.array([1,0,0])):
    # We can find the rotation matrix that rotates reference vector into this new direction

    '''
    LEGACY: Calculates rotation matrix first and then tried to convert to quaternion
    Seems to be an unnecessry pain. Rather let's convert straight to quaternion.
    # ref: https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d

    # c = cosine of angle (dot product)
    # s = sine of angle (magnitude of cross product)
    c = np.dot(x_direction, line_direction)
    v = np.cross(x_direction, line_direction)
    s = np.linalg.norm(v)
    v_x = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]) # the cross product matrix
    R = np.eye(3) + v_x + np.matmul(v_x, v_x)*((1-c)/(s**2))
    '''

    # ref: https://stackoverflow.com/questions/10236557/getting-quaternion-to-rotate-between-two-vectors
    # Comparing to ref. convention, v -> x_direction, w -> line_direction, u -> rotation_axis
    F = np.arccos(np.dot(reference_vector, direction_vector))/2
    # get rotation axis
    rotation_axis_unnormalized = np.cross(reference_vector ,direction_vector)
    rotation_axis = rotation_axis_unnormalized/np.linalg.norm(rotation_axis_unnormalized)
    q_complex = np.sin(F)*rotation_axis

    return np.hstack((q_complex.flatten(), np.cos(F)))

def make_pose_stamped(frame_id, position, quaternion):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.header.stamp = rospy.Time.now()

    # Populate position as midpoint of the line
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]

    # Populate the orientation as quaternion
    pose_msg.pose.orientation.x = quaternion[0]
    pose_msg.pose.orientation.y = quaternion[1]
    pose_msg.pose.orientation.z = quaternion[2]
    pose_msg.pose.orientation.w = quaternion[3]

    return pose_msg
