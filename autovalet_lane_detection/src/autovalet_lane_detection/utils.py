'''
Utilities file for the lane detector class

Changelog:
    eva   -   ??  - initial commit
    subbu - 23/10 - refactor 
'''

import rospy
import sensor_msgs.point_cloud2 as pcl2

from std_msgs.msg import Header

def publishCloud(cloud, cloud_frame, handle):
    header          = Header()
    header.stamp    = rospy.Time.now()
    header.frame_id = cloud_frame
    pcl_message     = pcl2.create_cloud_xyz32(header, cloud)
    handle.publish(pcl_message)
