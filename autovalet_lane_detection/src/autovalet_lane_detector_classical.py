#!/usr/bin/env python

import sys
sys.dont_write_bytecode = True
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image as pImage

# ROS
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber

class LaneDetector:

    def __init__(self, color_topic, depth_topic, cameraInfo_topic, laneCloud_topic):
        self.bridge    = CvBridge()
        self.color_sub = Subscriber(color_topic, Image)
        self.depth_sub = Subscriber(depth_topic, Image)

        # read camera info by looking up only one message
        self.camera        = rospy.wait_for_message(cameraInfo_topic, CameraInfo)
        self.laneCloud_pub = rospy.Publisher(laneCloud_topic, PointCloud2, queue_size=1)

        # Looking to synchronize both the topics within a 1/10th of a second
        self.ats = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=1, slop=0.5)
        self.ats.registerCallback(self.image_callback)

        # Define the camera matrix individual values for projection
        self.fx = self.camera.K[0]
        self.fy = self.camera.K[4]
        self.cx = self.camera.K[2]
        self.cy = self.camera.K[5]

        # define ROI
        self.ROI_UPPER_Y = 340
        self.ROI_RIGHT_X = 480

        # tracker
        self.tracker = None

    def image_callback(self, color_msg, depth_msg):
        # cvBridge image
        color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # lane detection algo
        center_line_coordinates = self.center_line_detection(color_img)               # px2
        """
        if center_line_coordinates is not None:
            center_line_cloud       = self.line2cloud(depth_img, center_line_coordinates) # px3
            lane_cloud              = self.interpolateAnotherLine(center_line_cloud)      # 2px3

            # publish point cloud
            self.publishLaneCloud(lane_cloud, depth_msg.header.frame_id)
        """

    def center_line_detection(self, img):
        print("callback", rospy.Time.now())
        # colorspace transformation
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS).astype(np.float)

        # hls thresholding for yellow
        # perfect bounds: straightway_2020: 12-20, 0.18-1, 0.9-1
        lower = np.array([12.0, 0.18*255, 0.90*255], dtype=np.uint8)
        upper = np.array([30.0, 1.00*255, 1.00*255], dtype=np.uint8)
        mask  = cv2.inRange(hls, lower, upper)
        th    = cv2.bitwise_and(rgb, rgb, mask=mask).astype(np.uint8)
        th    = cv2.cvtColor(th, cv2.COLOR_HLS2RGB)
        th    = cv2.cvtColor(th, cv2.COLOR_RGB2GRAY)
        th    = cv2.GaussianBlur(th, (5, 5), 0)
        _, th = cv2.threshold(th, 60, 255, cv2.THRESH_BINARY)
	th    = cv2.morphologyEx(th, cv2.MORPH_OPEN, (5,5))

        # extract ROI
        mask = np.zeros_like(th)
        mask[self.ROI_UPPER_Y:, :self.ROI_RIGHT_X] = 1
        roi = cv2.bitwise_and(th, th, mask=mask)

        # detect center line
        minLineLength, maxLineGap = 130, 20
        edges = cv2.Canny(roi, 100, 200, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, np.array([]), minLineLength, maxLineGap)

        # possible scenarios: 1. new line 2. tracking 3. lose track, using previous result
        max_len, scenario, X1, Y1, X2, Y2 = 0, 0, 0, 0, 0, 0
        if lines is not None: # 1. new line
            for line in lines:
                x1, y1, x2, y2 = line[0]
                line_len = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                if max_len < line_len:
                    max_len = line_len
                    X1, Y1, X2, Y2 = x1, y1, x2, y2
            scenario = 1
            bb = (X1, Y1, np.abs(X2-X1), np.abs(Y2-Y1))
            self.tracker = cv2.TrackerKCF_create()
            self.tracker.init(rgb, bb)
            self.lose_track_count = 0
        elif self.tracker is not None:
            success, bb = self.tracker.update(rgb)
            x, y, w, h = [int(v) for v in bb]
            if success: # 2. tracking
                scenario = 2
                X1, Y1, X2, Y2 = x, y, x+w, y-h
                self.lose_track_count = 0
            """
            else:
                self.lose_track_count += 1
                if self.lose_track_count < 10: # 3. lose track, using previous result
                    scenario = 3
            """

        # visualize
        self.visualize(img, th, scenario, X1, Y1, X2, Y2)

        """
        # coordinates and slope
        if (X1 + X2 + Y1 + Y2) != 0:
            slope = 1.0 * (Y2 - Y1) / (X1 - X2 + 0.0001) # assuming slope being positive, might have bug
            if X1 == X2:
                slope = 10000
                coordinates = [[X1, i] for i in range(Y2, Y1+1)]
            else:
                if slope > 1:
                    coordinates = [[X1+i, round(Y1-i*slope)] for i in range(X2-X1+1)]
                else:
                    coordinates = [[round(X1+i/slope), Y1-i] for i in range(Y1-Y2+1)]

            print("scenraio: ", scenario, " slope: ", slope, X1, Y1, X2, Y2)
        else:
            print("detection failed")
            return None

        return np.array(coordinates)
        """
        return None

    def line2cloud(self, depth, coordinates):
        # ref: Subbu's code
        u, v = coordinates[:, 0], coordinates[:, 1]
        x    = (u - self.cx) / self.fx
        y    = (u - self.cx) / self.fx

        z = depth[v, u] / 1000.0;
        x = np.multiply(x, z)
        y = np.multiply(y, z) - 2 # tmp fix accounted for map being gen w.r.t base_link

        cloud = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)))

        return cloud

    def interpolateAnotherLine(self, center_line_cloud):
        # extend each (x,y) along a normal vector of the center line
        # toward right, for lane_width meters
        lane_width = 3.66 # meters, US standard

        # approach 1
        """
        center_xy = center_line_cloud[:, :2]
        center_z  = center_line_cloud[:, 2]

        line_vec   = (np.roll(center_xy, -1, axis=0) - center_xy)[:-1]
        norm_vec   = np.hstack((line_vec[1].reshape(-1,1), -line_vec[0].reshape(-1,1)))
        norm_vec   = norm_vec / np.linalg.norm(line_vec, axis=1)

        right_line_xy = (center_xy + lane_width * norm_vec)[:-1]
        x, y          = right_line_xy[:, 0], right_line_xy[:, 1]
        z             = center_z[:-1]
        ###################################
        """

        # approach 2
        center_x = center_line_cloud[:, 0]
        center_y = center_line_cloud[:, 1]
        center_z = center_line_cloud[:, 2]

        # calculate the mean of the points
        center_mean = center_line_cloud.mean(axis=0)

        # Do an SVD on the mean-centered data.
        u, d, vh = np.linalg.svd(center_line_cloud - center_mean)
        orth_vec = np.array([vh[0][1], -vh[0][0]])
        norm_vec = orth_vec / np.linalg.norm(orth_vec)

        # extend the points with lane_width
        x = center_x + lane_width * norm_vec[0]
        y = center_y + lane_width * norm_vec[1]
        z = center_z
        ###################################

        # stack the cloud
        right_line_cloud = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)))
        lane_cloud       = np.vstack((center_line_cloud, right_line_cloud))

        return lane_cloud

    def visualize(self, img, th, scenario, X1, Y1, X2, Y2):
        res = np.stack((th, th, th), axis=-1)
        colorMap = {0: (255, 0, 0), 1: (0, 0, 255), 2: (0, 255, 0), 3: (255, 0, 255)}

        # draw line
        cv2.line(res, (X1, Y1), (X2, Y2), colorMap[scenario], 3)

        # draw rectangle on ROI
        h, w, _ = img.shape
        self.ROI_UPPER_LEFT = (0, self.ROI_UPPER_Y)
        self.ROI_LOWER_RIGHT = (self.ROI_RIGHT_X, h)
        res = cv2.rectangle(res, self.ROI_UPPER_LEFT, self.ROI_LOWER_RIGHT, colorMap[0], 2)

        # concatenate data & result
        concat = np.concatenate((img, res), axis=0)
        cv2.imshow("test", concat)
        cv2.waitKey(10)

    def publishLaneCloud(self, lane_cloud, depth_frame_id):
        header          = Header()
        header.stamp    = rospy.Time.now()
        header.frame_id = depth_frame_id
        lane_pcl        = pcl2.create_cloud_xyz32(header, lane_cloud)
        self.laneCloud_pub.publish(lane_pcl)

if __name__ == "__main__":
    rospy.init_node('lane_detector_node')

    color_topic      = "/frontCamera/color/image_raw"
    depth_topic      = "/frontCamera/aligned_depth_to_color/image_raw"
    #depth_topic      = "/frontCamera/depth/image_rect_raw"
    cameraInfo_topic = "/frontCamera/color/camera_info"
    laneCloud_topic  = "/lane/pointCloud"

    detector = LaneDetector(color_topic, depth_topic, cameraInfo_topic, laneCloud_topic)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
