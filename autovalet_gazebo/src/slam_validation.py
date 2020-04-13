#!/usr/bin/env python


import sys
import os


import rospy

from geometry_msgs.msg import PointStamped
import numpy as np
import xml.etree.ElementTree as ET

class Error_Calculator:
	def __init__(self,filename):
		self.sub  = rospy.Subscriber("/clicked_point",PointStamped,self.clicked_point_callback)
		# self.current_feed_sub = rospy.Subscriber("/subbu", Image, self.current_img_callback)

		self.map_frame_transform = np.array([[1.,0.,0.,18],
											  [0.,1.,0.,9],
											  [0.,0.,1.,0],
											  [0.,0.,0.,1]])
		self.i = 0
		self.num_points = 28
		self.dist = 0
		self.filename = filename
		self.box_points = self.parse_world(self.filename)
		self.corners = [["BL","TL"],
         				["BR","TR"],
				       	["BR","TR"],
				        ["BR","TR"],
				        ["BR","TR"],
				        ["BR","BL"],
				        ["TR","TL"],
				        ["TR","TL"],
				        ["BR","TR"],
				        ["TL","BL"],
				        ["TL","BL"],
				        ["TL","BL"],
				        ["TR","BR"],
				        ["TR","BR"]]
		self.corner_points = self.calc_corners(self.box_points,self.corners)

	# calculate box corner points given box centroids and directions
	# talk to kevin about this, he knows what it means... :)
	def calc_corners(self,points,directions):
	    points_list = []
	    for i in range(len(directions)):
	        # print(temp_point)
	        for j in range(len(directions[i])):
	            temp_point = points[i].copy()
	            temp_point[2] += 0.5
	            if directions[i][j] == "TR":
	                temp_point[0] += 0.5
	                temp_point[1] -= 0.5
	            if directions[i][j] == "BR":
	                temp_point[0] -= 0.5
	                temp_point[1] -= 0.5
	            if directions[i][j] == "TL":
	                temp_point[0] += 0.5
	                temp_point[1] += 0.5
	            if directions[i][j] == "BL":
	                temp_point[0] -= 0.5
	                temp_point[1] += 0.5
	            # print(temp_point)
	            points_list.append(temp_point)
	    new_points = np.array(points_list)
	    print(new_points)
	    return new_points

	# parse world file for the unit box centroid [x,y,z] poses
	def parse_world(self,filename):

	    tree = ET.parse(filename)
	    root = tree.getroot()
	    # print(root)
	    root = root[0]
	    # print(root)
	    pose_list= []
	    for x in root:
	        # print(model)
	        if x.tag == 'state':
	            # name = list(model.attrib.values())[0]
	            # print(model.attrib)
	            for child in x:
	                if child.tag == 'model':
	                    name = list(child.attrib.values())[0]
	                    if 'unit_box' in name:
	                        # print(name)
	                        for y in child:
	                            if y.tag == 'pose':
	                                text = y.text
	                                temp_pose = np.ones(4)
	                                text = text.split()
	                                temp_pose[0] = float(text[0])
	                                temp_pose[1] = float(text[1])
	                                temp_pose[2] = float(text[2])
	                                # print(temp_pose)
	                                pose_list.append(temp_pose)                                
	    pose_list = np.array(pose_list)
	    return pose_list

	def clicked_point_callback(self,data):
		map_x = data.point.x
		map_y = data.point.y
		map_z = data.point.z


		map_point = np.array([map_x,map_y,map_z,1])
		

		map_point = map_point[:3]
		print("generated",map_point)

		gt_point = np.dot(self.map_frame_transform, self.corner_points[self.i])

		gt_point = gt_point[:3]
		print("ground truth", gt_point)

		self.i += 1
		self.dist += np.linalg.norm(map_point - gt_point)

		avg = self.dist / self.i

		print("Average Error:", avg, "m")
		print("*********")
		if self.i == self.num_points:
			print("Error test completed")
			self.sub.unregister()


if __name__ == "__main__":

	rospy.init_node('test')



	calc = Error_Calculator('/home/kob51/catkin_ws/src/autovalet/autovalet_gazebo/worlds/slam_val.world')



	rospy.spin()