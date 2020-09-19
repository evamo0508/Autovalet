#!/usr/bin/env python

import numpy as np

# Include ROS libs
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

# Include messages
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from actionlib_msgs.msg import GoalStatusArray

def getCurrentPose():

    tf_buffer      = tf2_ros.Buffer(rospy.Duration(1200.0)) # Length of tf2 buffer (?)
    tf_listener    = tf2_ros.TransformListener(tf_buffer)


    transform = tf_buffer.lookup_transform("icp_odom", # target_frame_id
                                "map", #source frame
                                rospy.Time(0), #get the tf at first available time
                                rospy.Duration(1.0)) #timeout after 1

    current_pose = PoseStamped()
    current_pose.header = transform.header
    current_pose.pose.position.x = transform.transform.translation.x
    current_pose.pose.position.y = transform.transform.translation.y
    current_pose.pose.position.z = transform.transform.translation.z
    current_pose.pose.orientation = transform.transform.rotation

    return current_pose

def arePosesClose(pose1, pose2):
    result = True

    if result and pose1.pose.position.x != pose2.pose.position.x:
        result = False
    if result and pose1.pose.position.y != pose2.pose.position.y:
        result = False

    return result

class State:
    Waiting = 0
    Planning = 1
    Executing = 2
    Fail = 3
    Success = 4

class MoveBaseListener:
    def __init__(self):
        self.goal_sub       = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback)
        self.status_sub     = rospy.Subscriber("/move_base/status", GoalStatusArray, self.MBstatusCallback)
        
        self.goal_pose      = PoseStamped()
        self.start_pose     = PoseStamped()

        self.plan_state     = State.Waiting

    def printState(self):
        rospy.logwarn("---------")
        output = "Current State: "

        if self.plan_state == State.Waiting:
            output += "WAITING FOR NEXT GOAL"
        elif self.plan_state == State.Planning:
            output += "PLANNING PATH TO GOAL"
        elif self.plan_state == State.Executing:
            output += "EXECUTING PLAN"
        elif self.plan_state == State.Fail:
            output += "FAILED TO FIND A PLAN"
        elif self.plan_state == State.Success:
            output += "SUCCESSFULLY REACHED GOAL"

        rospy.logwarn(output)

    def goalCallback(self,goal_msg):
        

        self.plan_state = State.Planning

        self.goal_pose = goal_msg

        self.start_pose = getCurrentPose()

        self.printState()

        rospy.logwarn("Goal:")
        rospy.logwarn("Position")
        rospy.logwarn("  x: %f",self.goal_pose.pose.position.x)
        rospy.logwarn("  y: %f",self.goal_pose.pose.position.y)
        rospy.logwarn("  z: %f",self.goal_pose.pose.position.z)
        rospy.logwarn("Orientation");
        rospy.logwarn("  x: %f",self.goal_pose.pose.orientation.x)
        rospy.logwarn("  y: %f",self.goal_pose.pose.orientation.y)
        rospy.logwarn("  z: %f",self.goal_pose.pose.orientation.z)
        rospy.logwarn("  w: %f",self.goal_pose.pose.orientation.w)

    def MBstatusCallback(self,status_array_msg):

        # print(status_array_msg.__dict__)
        ACTIVE = 1
        SUCCEEDED = 3
        ABORTED = 4

        if status_array_msg.status_list:
            current_status = status_array_msg.status_list[-1]

            if current_status.status != ACTIVE and self.plan_state == State.Waiting:
                return

            if current_status.status == ACTIVE:
                if self.plan_state == State.Waiting:
                    self.plan_state = State.Planning
            
                elif self.plan_state == State.Planning:
                    current_pose = getCurrentPose()

                    if not arePosesClose(current_pose,self.start_pose):
                        self.plan_state = State.Executing
                    else:
                        return
                elif self.plan_state == State.Executing:
                    return

            elif current_status.status == SUCCEEDED:
                if self.plan_state == State.Waiting:
                    return
                else:
                    self.plan_state = State.Success
                    self.printState()
                    self.plan_state = State.Waiting
            
            elif current_status.status == ABORTED and self.plan_state != State.Waiting:
                if self.plan_state == State.Waiting:
                    return

                else:
                    self.plan_state = State.Fail
                    self.printState()
                    self.plan_state = State.Waiting

        else:
            return

        self.printState()


# # class AutoValet:

# #     def __init__():

# #         self.lane_detector = LaneDetector # import from eva's pkg

# #         self.next_pose = None

# #         self.parking_pose = None

# #         self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)



if __name__ == '__main__':

    # AV = AutoValet()

    rospy.init_node('planner')

    mbl = MoveBaseListener()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")