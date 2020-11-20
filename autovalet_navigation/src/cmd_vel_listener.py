import numpy as np

# Include ROS libs
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

# Include messages
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Twist
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from actionlib_msgs.msg import GoalStatusArray


class velocityListener:
    def __init__(self):
        self.max_x = 0
        self.max_theta = 0
        self.max_y = 0
        rospy.Subscriber("/cmd_vel", Twist, self.maxVel)
        self.count == 0

def maxVel(self,msg):
    self.count += 1

    if msg.linear.x > self.max_x:
        self.max_x = msg.linear.x

    if msg_linear.y > self.max_y:
        self.max_y = msg.linear.y

    if msg.angular.z > self.max_theta:
        self.max_theta = msg.angular.z
    
    if self.count % 50 == 0:
        print()
        print("max x", self.max_x)
        print("max y", self.max_y)
        print("max theta", self.max_theta)


if __name__ == '__main__':

    rospy.init_node('velocity_listener')
    

    vl = velocityListener()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

