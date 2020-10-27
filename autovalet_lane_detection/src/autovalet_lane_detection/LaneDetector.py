'''
Main class for lane detector

Author  : szuyum@andrew.cmu.edu
Date    : ???

Changelog:
    eva   -   ??  - inital commit
    subbu - 23/10 - refactor
'''

# python libs
import numpy as np
import cv2

# roslibs
import rospy

# ros messages
from sensor_msgs.msg import CameraInfo, PointCloud2

# custom libs
from utils import publishCloud

class LaneDetector:

    def __init__(self, colorInfo_topic, laneCloud_topic, egoLine_topic, hlsBounds, lineParams,debug=True):

        # Setup publishers
        self.laneCloud_pub = rospy.Publisher(laneCloud_topic, PointCloud2, queue_size=1)
        self.egoLine_pub   = rospy.Publisher(egoLine_topic, PointCloud2, queue_size=1)

        # read camera info by looking up only one message
        self.camera        = rospy.wait_for_message(colorInfo_topic, CameraInfo)

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

        # Lane and detector parameters
        lowerBound = hlsBounds['lowerbound']
        upperBound = hlsBounds['upperbound']
        self.lower_h, self.lower_l, self.lower_s = lowerBound['h'], lowerBound['l'], lowerBound['s']
        self.upper_h, self.upper_l, self.upper_s = upperBound['h'], upperBound['l'], upperBound['s']
        self.minLineLength = lineParams['minLineLength']
        self.maxLineGap = lineParams['maxLineGap']
        self.lane_width = 3.5

        self.debug = debug

    def detectLaneRGBD(self, color_img, depth_img):
        # lane detection algo
        center_line_coordinates = self.center_line_detection(color_img)                # px2
        try:
            center_line_cloud = self.line2cloud(depth_img, center_line_coordinates)    # px3
            norm_vec          = self.findNormalVectorInCloud(center_line_cloud)

            # Generate the right line for enforcing costmap constraints
            right_line        = self.interpolateLine(center_line_cloud, norm_vec, self.lane_width)
            lane_cloud        = np.vstack((center_line_cloud, right_line)) # 2px3

            # Generate the ego line for goal generation
            ego_line          = self.interpolateLine(center_line_cloud, norm_vec, self.lane_width/2) # px3

            # Publish all our computed clouds
            publishCloud(lane_cloud, self.camera.header.frame_id, self.laneCloud_pub)
            publishCloud(ego_line, self.camera.header.frame_id, self.egoLine_pub)
            # maybe publish center line alone too?

            return lane_cloud, ego_line

        except:
            if self.debug:
                print("empty center line")
            return None, None

    def center_line_detection(self, img):
        # colorspace transformation
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS).astype(np.float)

        # bounds & mask for hls thresholding of color yellow
        lower = np.array([self.lower_h, self.lower_l*255, self.lower_s*255], dtype=np.uint8)
        upper = np.array([self.upper_h, self.upper_l*255, self.upper_s*255], dtype=np.uint8)

        mask  = cv2.inRange(hls, lower, upper)

        # hls thresholding
        th    = cv2.bitwise_and(rgb, rgb, mask=mask).astype(np.uint8)

        # post-filtering to remove noise
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
        edges = cv2.Canny(roi, 100, 200, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, np.array([]), self.minLineLength, self.maxLineGap)

        # possible scenarios: 0. no line detected 1. new line 2. tracking
        max_len, scenario = 0, 0
        X1, Y1, X2, Y2 = 0, 0, 0, 0
        if lines is not None: # 1. new line
            for line in lines:
                x1, y1, x2, y2 = line[0]
                line_len = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                if max_len < line_len:
                    max_len = line_len
                    X1, Y1, X2, Y2 = x1, y1, x2, y2
            scenario = 1
            bbox = (X1, Y1, np.abs(X2-X1), np.abs(Y2-Y1))
            self.tracker = cv2.TrackerKCF_create()
            self.tracker.init(rgb, bbox)
            self.lose_track_count = 0
        elif self.tracker is not None:
            success, bbox = self.tracker.update(rgb)
            x, y, w, h = [int(v) for v in bbox]
            if success: # 2. tracking
                scenario = 2
                X1, Y1, X2, Y2 = x, y, x+w, y-h
                Y1 = min(Y1, img.shape[0] - 1)
        if scenario == 0:
            return None

        # lines detected, calculate coordinates and slope
        if X1 == X2: # vertical line
            coordinates = [[X1, i] for i in range(Y2, Y1+1)]
        else:
            slope = 1.0 * (Y2 - Y1) / (X1 - X2)
            # assuming slope being positive, might have bug
            # sign of slope is flipped here b/c origin is at top-left corner of image
            if slope > 1:
                coordinates = [[X1+i, int(Y1-i*slope)] for i in range(X2-X1)]
            else:
                coordinates = [[int(X1+i/slope), Y1-i] for i in range(Y1-Y2)]

        return np.array(coordinates)

    def line2cloud(self, depth, coordinates):
        # ref: Subbu's code in goal_generation/scripts/transfer_lane.py
        #  |u|   |fx 0 cx| |x|
        # s|v| = |0 fy cy|*|y|, where x,y,z is in rgb cam's frame.
        #  |1|   |0  0  1| |z|, x point to the right, y points downward, z points forward

        u, v = coordinates[:, 0], coordinates[:, 1]
        x    = (u - self.cx) / self.fx
        y    = (v - self.cy) / self.fy

        z = depth[v, u] / 1000.0; # unit: mm -> m
        x = np.multiply(x, z)
        y = np.multiply(y, z) - 2 #: tmp fix accounted for map being gen w.r.t base_link

        x = x[np.nonzero(z)]
        y = y[np.nonzero(z)]
        z = z[np.nonzero(z)]

        cloud = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)))

        return cloud

    def findNormalVectorInCloud(self, center_line_cloud):
        # calculate the mean of the points
        center_mean = center_line_cloud.mean(axis=0)

        # Do an SVD on the mean-centered data.
        # vh[0] corresponds to the principal axis. i.e. line vector
        # vh[-1] corresponds to the normal vector of the center line, but not unique since this is a 3D world.
        # taking (z, -x) s.t. taking inner product with (x, z) = 0
        _, _, vh = np.linalg.svd(center_line_cloud - center_mean)
        orth_vec = np.sign(vh[0][2]) * np.array([vh[0][2], -vh[0][0]])
        norm_vec = orth_vec / np.linalg.norm(orth_vec)

        return norm_vec

    def interpolateLine(self, center_line_cloud, norm_vec, move_dist):

        # Extend each pcl along a normal vector of the center line
        # for lane_width meters in rgb cam frame.

        center_x = center_line_cloud[:, 0]
        center_y = center_line_cloud[:, 1]
        center_z = center_line_cloud[:, 2]

        # extend the points with lane_width
        x = center_x + move_dist * norm_vec[0]
        y = center_y
        z = center_z + move_dist * norm_vec[1]

        moved_line_cloud = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)))

        return moved_line_cloud
