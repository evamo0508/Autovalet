#!/usr/bin/env python
'''
Script runs an approximate time policy message filter to catch
depthmap and segmentation maps at the same time and produces a
segmented lane cloud and publishes it

Author  : Subramanian Krishnan (subramak@andrew.cmu.edu)
Date    : 09 September 2020

Changelog:
    subbu - 9/07 - Initial commit
    subbu - 9/07 - Lane seg overlaid on depthmap
    eva/subbu - 9/12 - Fixed fov
    subbu - 9/14 - lane line extraction from segmap
    subbu - 9/18 - Depth registration added
'''
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pcl2
import cv2

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber

class transfer:

  def __init__(self, segmap_topic, depthmap_topic, cameraInfo_topic, laneCloud_topic):
    self.bridge = CvBridge()
    self.depth_sub  = Subscriber(depthmap_topic, Image)
    self.segmap_sub = Subscriber(segmap_topic, Image)

    # Read camera info by looking up only one message
    self.camera = rospy.wait_for_message(cameraInfo_topic, CameraInfo)
    self.lane_cloud_pub = rospy.Publisher(laneCloud_topic, PointCloud2, queue_size=1)

    # Looking to synchronize both the topics within a 1/10th of a second
    self.ats = ApproximateTimeSynchronizer([self.depth_sub, self.segmap_sub], queue_size=2, slop=0.1)
    self.ats.registerCallback(self.callback)

    # Define the camera matrix individual values for projection
    self.fx     = self.camera.K[0]
    self.fy     = self.camera.K[4]
    self.cx     = self.camera.K[2]
    self.cy     = self.camera.K[5]

    # Define kernel for dilation of lanes
    self.dilation_kernel = np.ones((2,2),np.uint8)

  def callback(self,depthmap, segmap):
    # process image
    depth_image = self.bridge.imgmsg_to_cv2(depthmap, desired_encoding="passthrough")
    segmap_image = self.bridge.imgmsg_to_cv2(segmap, desired_encoding="passthrough")

    # Close the segmap to remove small holes in the segmented lane
    segmap_processed = cv2.morphologyEx(segmap_image, cv2.MORPH_CLOSE, self.dilation_kernel)
    # Generate new mask with lane boundary
    lane_lines = cv2.Canny(segmap_processed, 100, 200)
    # Perform morphological opening to increase width of lane line
    lane_lines = cv2.dilate(lane_lines,self.dilation_kernel,iterations = 1)

    # Overlay mask
    lane_depth_image = cv2.bitwise_and(depth_image, depth_image, mask = lane_lines/255)

    # Project depth back to 3D
    x,y,z = self.convert_depth_frame_to_pointcloud(lane_depth_image)

    lane_header = Header()
    # Populate header (Note: Frame ID is same as depth map's frame)
    lane_header.stamp    = rospy.Time.now()
    lane_header.frame_id = depthmap.header.frame_id

    lane_points = np.hstack((x.reshape(-1,1), y.reshape(-1,1), z.reshape(-1,1)))
    lane_pcl    = pcl2.create_cloud_xyz32(lane_header, lane_points)
    self.lane_cloud_pub.publish(lane_pcl)

  def convert_depth_frame_to_pointcloud(self, depth_image):
	"""
	Convert the depthmap to a 3D point cloud
	Parameters:
	-----------
	depth_frame 	 	 : rs.frame()
						   The depth_frame containing the depth map
	camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
	Return:
	----------
	x : array
		The x values of the pointcloud in meters
	y : array
		The y values of the pointcloud in meters
	z : array
		The z values of the pointcloud in meters
	"""

	[height, width] = depth_image.shape

	nx = np.linspace(0, width-1, width)
	ny = np.linspace(0, height-1, height)
	u, v = np.meshgrid(nx, ny)
	x = (u.flatten() - self.cx)/self.fx
	y = (v.flatten() - self.cy)/self.fy

	z = depth_image.flatten() / 1000.0;
	x = np.multiply(x,z)
	y = np.multiply(y,z)

	x = x[np.nonzero(z)]
	y = y[np.nonzero(z)]
	z = z[np.nonzero(z)]

	return x, y-2, z

def main():

    rospy.init_node("lane_transfer", anonymous=False)

    segmap      = '/lane/ground_truth'
    depthmap    = '/depth_registered/image_rect'
    camera_info = '/frontCamera/color/camera_info'
    lane_cloud  = '/lane/pointcloud'

    lane_transfer = transfer(segmap, depthmap, camera_info, lane_cloud)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
