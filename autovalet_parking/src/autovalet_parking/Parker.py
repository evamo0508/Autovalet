import numpy as np

# Include ROS libs
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import os

# Include messages
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from gazebo_msgs.msg import LinkStates

class Parker:
    def __init__(self, goal_topic, tag_topic, goal_frame_id, husky_frame_id, aruco_frame_name,debug=True):

        # for accumulating tag poses
        self.tag_dist_tol = 4.0
        self.num_tag_to_ransac = 10
        self.tfArray = []

        # Setup listeners and talkers
        self.aruco_subscriber = rospy.Subscriber(tag_topic, PoseStamped, self.collectTagPoses)

        self.tf_buffer        = tf2_ros.Buffer(rospy.Duration(1200.0)) # Length of tf2 buffer (?)
        self.tf_listener      = tf2_ros.TransformListener(self.tf_buffer)
        self.aruco_frame_name = aruco_frame_name
        self.goal_frame_id    = goal_frame_id
        self.husky_frame_id   = husky_frame_id

        self.parkingBroadcaster = tf.TransformBroadcaster()

        # for comparing tag with centerline position
        self.centerline_midpt = None

        # helper const & bool
        self.costmap_height              = rospy.get_param('/move_base/global_costmap/height')
        self.first_goal_in_costmap       = False
        self.first_goal_is_close_meter   = 0.8
        self.debug = debug

        # parking goals
        self.goal1 = PoseStamped()
        self.goal2 = PoseStamped()

        # ground truth parking goal calculated from Gazebo links
        self.gt_goal2 = PoseStamped()

        # flag to know when we are ready to defer to parker's goals
        self.ready = False

        # Validation data members
        self.park_direction = None
        self.target_pose    = None
        self.reached_pose   = None
        self.aruco_tag_gt   = None
        self.base_link_gt   = None
        self.gt_topic       = '/gazebo/link_states'


    def collectTagPoses(self, tag_pose):
        '''
        Take in numOfTags tag poses and do RANSAC in the end to avoid outliers
        '''
        dist2tag = np.linalg.norm(np.array([tag_pose.pose.position.x, tag_pose.pose.position.y]))
        print dist2tag
        # print tag_pose.pose.position
        if dist2tag > self.tag_dist_tol or len(self.tfArray) < self.num_tag_to_ransac: # collect tag poses
            tf = self.tf_buffer.lookup_transform(self.goal_frame_id, # map
                                    self.aruco_frame_name, # parking_spot
                                    rospy.Time(0), # get the tf at first available time
                                    rospy.Duration(1.0)) # timeout after 1
            self.tfArray.append(tf)
        elif not self.ready: # perform RANSAC on transformArray
            self.tag_tf = self.tagPoseRANSAC()
            self.setParkDirection()
            self.calculateGoals()
            self.ready = True
            self.aruco_subscriber.unregister()
            # os.system("rosnode kill ARUCO")

    def getTagTF(self):
        return self.tag_tf

    def isReady(self):
        return self.ready

    def getParkingPoses(self):
        return [self.goal1,self.goal2]

    def setParkDirection(self):
        tag_to_base_link = self.tf_buffer.lookup_transform(self.husky_frame_id,
                                                           self.aruco_frame_name,
                                                           rospy.Time(0),
                                                           rospy.Duration(1.0))
        if self.centerline_midpt.y > tag_to_base_link.transform.translation.y:
            self.park_direction = "right"
        else:
            self.park_direction = "left"

    def calculateGoals(self):
        # if apriltag is to the right of the line
        if self.park_direction == "right":
            pos1 = [0, 0.5, 3.0] # position relative to detected tag pose
            rot1 = [0, -np.pi/4, -np.pi/4]

            pos2 = [0, 0.5, 5.2]
            rot2 = [0, -np.pi/2, -np.pi/2]

        # if apriltag is to the left of the line
        elif self.park_direction == "left":
            pos1 = [0, 0.5, -1.0]
            rot1 = [0, np.pi/4, -np.pi/4]

            pos2 = [0, 0.5, -5]
            rot2 = [0, np.pi/2, np.pi/2]

        self.goal1 = self.generateParkingGoal(self.tag_tf, pos1, rot1, 1)
        self.goal2 = self.generateParkingGoal(self.tag_tf, pos2, rot2, 2)

    def generateParkingGoal(self, tf, pos, rot, goal_num):
        '''
        waypoint published to movebase
        '''

        name = "parking_goal" + str(goal_num)
        tag2waypoint = PoseStamped()
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
        self.parkingBroadcaster.sendTransform((goal.pose.position.x, goal.pose.position.y, goal.pose.position.z),
                                               (goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w),
                                               rospy.Time.now(),
                                               name,
                                               self.goal_frame_id)
        goal.pose.position.z = 0 # Enforce 2D nav constraint
        # Correct the orientation of the tag (enforce roll and pitch zero)
        goal.pose.orientation = self.orientationCorrection(goal.pose.orientation)

        self.parkingBroadcaster.sendTransform((goal.pose.position.x, goal.pose.position.y, goal.pose.position.z),
                                               (goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w),
                                               rospy.Time.now(),
                                               name + "_proj",
                                               self.goal_frame_id)

        return goal

    def orientationCorrection(self, quat):
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

    def distToGoal(self, goal):
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

    def tagPoseRANSAC(self):
        '''
        distance between quaternions is calculated based on the formula here:
        https://math.stackexchange.com/questions/90081/quaternion-distance
        '''
        quatArray = []
        self.tfArray = self.tfArray[-self.num_tag_to_ransac:]
        for i in range(len(self.tfArray)):
            quat = self.tfArray[i].transform.rotation
            quatArray.append(np.array([quat.x, quat.y, quat.z, quat.w]))
        votes = []
        for i in range(len(quatArray)):
            vote = 0
            for j in range(len(quatArray)):
                diff = np.abs(1 - np.dot(quatArray[i], quatArray[j]) ** 2)
                vote = vote + 1 if diff < 1e-3 else vote #TODO
            votes.append(vote)
        # pick the pose with the most inliers
        tf = self.tfArray[votes.index(max(votes))]

        return tf

    def calculate_error(self):

        # Get the ground truth and actual poses
        gt = rospy.wait_for_message(self.gt_topic, LinkStates)
        self.target_pose = self.get_target_pose(gt)
        self.reached_pose = self.get_reached_pose(gt)

        # Error in x and y in cms
        trans_err = (self.target_pose[:-1] - self.reached_pose[:-1])*100
        # Error in orientation (yaw) in degrees
        # make sure the orientation is the same in hardware so that the "270" works as well
        align_tag_to_goal = -90 if self.park_direction == "left" else 90
        yaw_err   = np.abs((self.target_pose[2]*180/3.14 + align_tag_to_goal) - self.reached_pose[2]*180/3.14) #comparing the aruco tag (rotated by 270deg) with husky's yaw
        print 'yaw err b4 wrapping', yaw_err
        if yaw_err > 180:
            yaw_err -= 180
        if yaw_err > 300:
            yaw_err -= 360
        print "Target pose (x,y,theta): ", self.target_pose[:-1]*100, self.target_pose[2]*180/3.14 + align_tag_to_goal
        print "Reached pose (x,y,theta): ", self.reached_pose[:-1]*100, self.reached_pose[2]*180/3.14
        print "Translation error (cms): ", trans_err
        print "Orientation error (deg): ", yaw_err
        print "Net error (norm of the error)", np.linalg.norm(trans_err)

    def get_target_pose(self, gt):

        self.aruco_tag_gt    = gt.pose[-18] # name 'aruco_visual_marker_7::marker' <to-do> find the name! Don't HC
        if self.park_direction == "right":
            compensation = np.array([0.0, -4.0 ,0.0])
        if self.park_direction == "left":
            compensation = np.array([0.0, 4.0, 0.0])
        aruco_tag_gt_2d = np.array([self.aruco_tag_gt.position.x , self.aruco_tag_gt.position.y, self.quat_to_euler(self.aruco_tag_gt.orientation)[2]]) - compensation
        return aruco_tag_gt_2d

    def get_reached_pose(self, gt):
        self.base_link_gt    = gt.pose[-5] # base_link w.r.t the gazebo world origin
        base_link_gt_2d = np.array([self.base_link_gt.position.x, self.base_link_gt.position.y, self.quat_to_euler(self.base_link_gt.orientation)[2]])

        return base_link_gt_2d

    def quat_to_euler(self, q):
        # euler indexing as RPY
        quaternion = (q.x, q.y, q.z, q.w)
        euler = euler_from_quaternion(quaternion)
        return euler
