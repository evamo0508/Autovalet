#!/usr/bin/env python

import numpy as np
import sys

# Include ROS libs
import rospy
import tf2_ros
import smach
from tf2_geometry_msgs import do_transform_pose
import rospkg
import rosparam
import yaml

# Include messages
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from actionlib_msgs.msg import GoalStatusArray

# Include image helpers
from PIL import Image as pImage
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Include autovalet nodes
from autovalet_lane_detection.LaneDetector import LaneDetector
from autovalet_navigation.move_base_listener import MoveBaseListener
from autovalet_husky.find_parking_spot import parking_spot
from autovalet_goal_generation.goal_generator import goal_generator


# struct to define states
class State:
    START = 0
    SEND_GOAL = 1
    PLANNING = 2
    PARK = 3
    FINISH = 4

class AutoValet:
    def __init__(self,sim):
        self.sim = sim

        # moveBaseListener setup #############################
        self.moveBaseListener = MoveBaseListener(debug=False)

        # laneDetector setup ##################################
        self.colorInfo_topic = "/frontCamera/color/camera_info"
        self.laneCloud_topic = "/lane/pointCloud"
        self.egoLine_topic   = "/lane/egoLine"
        self.color_topic     = "/frontCamera/color/image_raw"
        self.depth_topic     = "/depth_registered/image_rect"
        self.bridge          = CvBridge()
        self.color_sub       = Subscriber(self.color_topic,Image)
        self.depth_sub       = Subscriber(self.depth_topic,Image)
        self.ats             = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=2, slop=0.5)
        self.ego_line        = None
        self.depth_frame_id  = None
        self.depth_img       = None
        self.color_img       = None
        self.ats.registerCallback(self.registered_image_callback)

        # time tracker for publishing goals at a lower rate
        self.previous_time = rospy.get_time()
        
        self.laneDetector = self.init_detector(self.colorInfo_topic, 
                                               self.laneCloud_topic,
                                               self.egoLine_topic)
        
        # goal gen setup ##########################################################
        self.goal_topic         = "/move_base_simple/goal"
        self.map_frame          = "map"
        self.goal_pub           = rospy.Publisher(self.goal_topic,PoseStamped,queue_size=1)
        self.goalGenerator      = goal_generator(self.map_frame)
        self.current_goal       = PoseStamped()

        # parking setup ###########################################################
        # tag_topic - name of the topic in which the pose of the marker is being published
        # husky_frame is the name of the husky's frame we want to keep track of
        # aruco_frame_name is the NAME OF THE FRAME PUBLISHED AS THE ARUCO TAG
        self.tag_topic          = '/ARUCO/pose'
        self.husky_frame        = 'base_link'
        self.aruco_frame_name   = 'parking_spot' #'aruco_marker_frame' or 'parking_spot'
                            # (Needs to be same as what is set in aruco launcher)
        
        self.parker = parking_spot(self.goal_topic,
                                   self.tag_topic,
                                   self.map_frame,
                                   self.husky_frame,
                                   self.aruco_frame_name,
                                   debug=False)
        
        # State Machine variables
        self.current_state = State.START
        self.prev_state = None

    # helper fxn to load the correct lane detection params and initialize LaneDetector class
    def init_detector(self, colorInfo_topic, laneCloud_topic, egoLine_topic):
        yaml_path = rospkg.RosPack().get_path('autovalet_lane_detection')
        yaml_path += "/config/"
        if self.sim:
            yaml_path += "sim_params.yaml"
        else:
            yaml_path += "hardware_params.yaml"
        
        f = open(yaml_path,'r')
        yaml_file = yaml.load(f)
        f.close()
        rosparam.upload_params('/lane_detection/',yaml_file)

        hlsBounds = rospy.get_param('/lane_detection/hlsbounds')
        lineParams = rospy.get_param('/lane_detection/lineparams')

        LD = LaneDetector(colorInfo_topic, 
                          laneCloud_topic, 
                          egoLine_topic, 
                          hlsBounds, 
                          lineParams,
                          debug=False)

        return LD

    # callback for image messages. this is what keeps the system moving forward, as processState gets
    # called everytime we get a new img
    def registered_image_callback(self, color_msg, depth_msg):
        # cvBridge image
        self.color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        self.depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        self.depth_frame_id = depth_msg.header.frame_id
        
        # if we're not in the PARK state, detect the lane and publish
        if self.current_state != State.PARK:
            # lane detection algo
            _, self.ego_line = self.laneDetector.detectLaneRGBD(self.color_img, self.depth_img)

        self.processState()
    
    def sendGoal(self):
        # generate goal from the egoline
        # if((rospy.get_time() - self.previous_time) > 2 and self.ego_line is not None):
        if self.ego_line is not None:
            self.current_goal = self.goalGenerator.generate_goal_from_egoline(self.ego_line, self.depth_frame_id)
            self.goal_pub.publish(self.current_goal)
            self.previous_time = rospy.get_time()

    def processState(self):
        # START state ############################
        if self.current_state == State.START:
            self.printState()
            self.prev_state = State.START
            self.current_state = State.SEND_GOAL

        # SEND_GOAL state #############################
        elif self.current_state == State.SEND_GOAL:
            # get goal from lane detector and publish
            self.sendGoal()
            self.printState()

            # move to PLANNING state
            self.prev_state = self.current_state
            self.current_state = State.PLANNING
            self.prev_time = rospy.get_time()

        # PLANNING state #######################
        elif self.current_state == State.PLANNING:
            # check if we are ready for parking (tag buffer is filled)
            # if so, move to PARK state
            if self.parker.count >= self.parker.tag_buffer_size:
                self.prev_state = self.current_state
                self.current_state = State.PARK

            # if it's been 2 secs since last sent goal (allow for processing time) AND we're within 2 m of last goal,
            # move to the SEND_GOAL state
            elif rospy.get_time() - self.prev_time > 2 and self.parker.dist_to_first_goal(self.current_goal) <= 2:
                self.prev_state = self.current_state
                self.current_state = State.SEND_GOAL
            
            # keep planning, only print if it's your first time entering this fxn in the PLANNING state
            else:
                if self.prev_state != State.PLANNING:
                    self.printState()
                    self.prev_state = self.current_state

        # PARK state ############################
        elif self.current_state == State.PARK:

            # printState() the first time you enter this state
            if self.prev_state != State.PARK:
                # rospy.ServiceProxy("/move_base/clear_costmaps", {})
                self.printState()
                self.prev_state = self.current_state

            # if the moveBaseListener is waiting for a new goal, that means we've reached the last goal we've
            # sent, so we're done!
            if self.moveBaseListener.getState() == "waiting":
                self.prev_state = self.current_state
                self.current_state = State.FINISH
        
        # FINISH state #############################
        # TODO - print parking error and time here
        elif self.current_state == State.FINISH:
            if self.prev_state != State.FINISH:
                self.printState()
                self.prev_state = self.current_state
        
    # fxn to print pretty log msgs (roswarn just cause yellow is easier to read imo)
    def printState(self):
        rospy.logwarn("---------")
        if self.current_state == State.START:
            rospy.logwarn("Starting AutoValet state machine...")
        elif self.current_state == State.SEND_GOAL:
            rospy.logwarn("Sending goal...")
            rospy.logwarn("Position")
            rospy.logwarn("  x: %f",self.current_goal.pose.position.x)
            rospy.logwarn("  y: %f",self.current_goal.pose.position.y)
            rospy.logwarn("  z: %f",self.current_goal.pose.position.z)
            rospy.logwarn("Orientation");
            rospy.logwarn("  x: %f",self.current_goal.pose.orientation.x)
            rospy.logwarn("  y: %f",self.current_goal.pose.orientation.y)
            rospy.logwarn("  z: %f",self.current_goal.pose.orientation.z)
            rospy.logwarn("  w: %f",self.current_goal.pose.orientation.w)
        elif self.current_state == State.PLANNING:
            rospy.logwarn("Planning...")
        elif self.current_state == State.PARK:
            rospy.logwarn("Executing parking maneuver...")
        elif self.current_state == State.FINISH:
            rospy.logwarn("Parking maneuver completed!")


if __name__ == '__main__':

    rospy.init_node('AUTOVALET')

    if sys.argv[-1] == "True" or sys.argv[-1] == "true":
        sim = True
    else:
        sim = False

    AV = AutoValet(sim)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")