#!/usr/bin/env python
"""
tf2 broadcaster node
This node broadcasts couple of tf2 frames and ensures these frames can
be updated using a dynamic reconfigure server. Primarily intended to find
tf between velodyne and d435 in the AutoValet project.

Usage: rosrun autovalet_husky vlp_cam_calib.py <parent_frame> <child_frame>
Run Rviz to visualize the tf moving around. Overwrite XYZ-RPY when clouds align.

author: subramak@andrew.cmu.edu
Date  : 4 Sep 2020

Changelog:
    subbu - 09/04 - Initial commit

"""

# references:
# 1. https://github.com/azhar92/scooter-mode
# 2. https://answers.ros.org/question/333820/use-tf2-within-a-dynamic-reconfigure-server/

## Import standard library packages ##
import sys

## Import ros packages ##
import rospy
from dynamic_reconfigure.server import Server
from autovalet_husky.cfg import lidar_camera_tfConfig
import tf_conversions
import tf
import tf2_ros
import geometry_msgs.msg

#################################################
## tf2_broadcaster class ########################
#################################################
class tf2_broadcaster():
    def __init__(self, parent_frame="velodyne", child_id="backCamera_link"):
        """Calib_frame_tf2_broadcaster class initialization.

        Parameters
        ----------
        parent_frame : str, optional
            Name of the parent frame, by default "panda_link0"
        child_id : str, optional
            Name of the child frame, by default "calib_frame"
        """

        ## Retrieve parameters ##
        self.parent_frame = parent_frame
        self.child_id = child_id

        ## Create static broadcster ##
        self.static_br = tf2_ros.StaticTransformBroadcaster()

        ## Initialize transform broardcaster ##
        self.timer = rospy.Timer(rospy.Duration(1.0/10.0),
                                 self.tf2_broadcaster_callback)
        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = geometry_msgs.msg.TransformStamped()

        ## Initialize dynamic reconfigure server ##
        self.srv = Server(lidar_camera_tfConfig, self.dync_reconf_callback)

    def dync_reconf_callback(self, config, level):
        """Dynamic reconfigure callback function.

        Parameters
        ----------
        config : dict
            Dictionary containing the dynamically reconfigured parameters.
        level : int
            A bitmask which will later be passed to the dynamic reconfigure callback.
            When the callback is called all of the level values for parameters that have
            been changed are ORed together and the resulting value is passed to the callback.
        """

        ## Print information ##
        rospy.loginfo("""Reconfigure Request: {x_pos}, {y_pos},\
            , {z_pos}, {yaw}, {pitch} & {roll}""".format(**config))

        ## Read dynamic parameter values ##
        self.x_pos = config["x_pos"]
        self.y_pos = config["y_pos"]
        self.z_pos = config["z_pos"]
        self.yaw = config["yaw"]
        self.pitch = config["pitch"]
        self.roll = config["roll"]

        quat = tf.transformations.quaternion_from_euler(
            float(self.yaw), float(self.pitch), float(self.roll), axes='rzyx')
        print(quat)

        ## Update parameters in the parameter server ##
        rospy.set_param('x_pos', config["x_pos"])
        rospy.set_param('y_pos', config["y_pos"])
        rospy.set_param('z_pos', config["z_pos"])
        rospy.set_param('yaw', config["yaw"])
        rospy.set_param('pitch', config["pitch"])
        rospy.set_param('roll', config["roll"])

        ## Return possibly updated configuration ##
        return config

    def tf2_broadcaster_callback(self, event=None):
        """TF2 broadcaster callback function. This function broadcast the tf2 frames.

        Parameters
        ----------
        event : rospy.timer.TimerEvent, optional
            Structure passed in provides you timing information that can be useful when debugging or profiling. , by default None
        """
        ## Generate TF message ##
        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.header.frame_id = self.parent_frame
        self.tf_msg.child_frame_id = self.child_id
        self.tf_msg.transform.translation.x = self.x_pos
        self.tf_msg.transform.translation.y = self.y_pos
        self.tf_msg.transform.translation.z = self.z_pos
        quat = tf.transformations.quaternion_from_euler(
            float(self.yaw), float(self.pitch), float(self.roll), axes='rzyx')
        self.tf_msg.transform.rotation.x = quat[0]
        self.tf_msg.transform.rotation.y = quat[1]
        self.tf_msg.transform.rotation.z = quat[2]
        self.tf_msg.transform.rotation.w = quat[3]

        ## Send tf message ##
        self.br.sendTransform(self.tf_msg)

#################################################
## Main script ##################################
#################################################
if __name__ == "__main__":

    ## Initialize TF2 broadcaster node ##
    rospy.init_node('Lidar_TF_tester', anonymous=False)

    ## Check if enough arguments are supplied ##
    if len(sys.argv) < 3:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './static_calib_frame_tf2_broadcaster.py '
                     'frame_id child_frame_name')
        sys.exit(0)
    else:

        ## Check if frame name is valid ##
        if sys.argv[2] == 'world':
            rospy.logerr('Your frame cannot be named "world"')
            sys.exit(0)

    ## Initialize calib frame broadcaster ##
    tf2_broadcaster(sys.argv[1], sys.argv[2])

    ## Spin forever ##
    rospy.spin()
