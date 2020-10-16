#!/usr/bin/env python

import numpy as np

# Include ROS libs
import rospy
import tf2_ros
import smach
from tf2_geometry_msgs import do_transform_pose

from autovalet_lane_detection.autovalet_lane_detector_sim import LaneDetector
from autovalet_navigation.move_base_listener import MoveBaseListener

# Include messages
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from actionlib_msgs.msg import GoalStatusArray

import smach

class GetGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['publish'])

    def execute(self,userdata):
        # goalgen.getgoal()
        # AutoValet.
            self.park = True
            return 'traverse'

            ## STATE MACHINE INSIDE TRAVERSE
            # if no goal --> get goal from goal_gen and publish
            # if not close to goal, circle back
            # if close to goal, get goal from goal_gen and publish
            



# lowercase --> transition label
# upppercase --> state to transition to (state labels defined in StateMachine.add)
class Traverse(smach.State):
    def __init__(self):
        print("begin traversing garage")
        smach.State.__init__(self, outcomes=['traverse','park'])
        self.park = False

    def execute(self,userdata):
        # if getParkingGoal == True:
        if self.park:

            return 'park'
        else:
            # sendGoalPose
            self.park = True
            return 'traverse'

            ## STATE MACHINE INSIDE TRAVERSE
            # if no goal --> get goal from goal_gen and publish
            # if not close to goal, circle back
            # if close to goal, get goal from goal_gen and publish
            
    
class Park(smach.State):
    def __init__(self):
        print("begin parking maneuver")
        smach.State.__init__(self, outcomes=['park','finish'])
        self.finished = False
    
    def execute(self,userdata):
        # if parkingComplete() == True:
        if self.finished:
            return 'finish'
        else:
            self.finished = True
            return 'park'

            ## STATE MACHINE INSIDE PARK
            # if no first 

# class Finish(smach.State):
#     def __init__(self):
#         print("Parked!")

#     def execute(self)


class AutoValet:
    def __init__(self):
        # self.laneDetector = LaneDetector()
        # self.goalGenerator = GoalGenerator()
        # self.parkingGenerator = parkingGenerator()

        self.sm = smach.StateMachine(outcomes=['finish'])

        self.sm.userdata.pusher = afsjlkasfdjlk
        
        with self.sm:
            smach.StateMachine.add('TRAVERSE',Traverse(),transitions={'park':'PARK','traverse':'TRAVERSE'})
            smach.StateMachine.add('PARK',Park(),transitions={'park':'PARK','finish':'finish'})

        self.currentGoal = PoseStamped()
        self.sm.execute()



if __name__ == '__main__':

    AV = AutoValet()

    


    # rospy.init_node('state_machine')

    # mbl = MoveBaseListener()



    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")