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

		if "clearpath" not in filename:

			self.map_frame_transform = np.array([[1.,0.,0.,18],
												  [0.,1.,0.,9],
												  [0.,0.,1.,-0.132272858645],
												  [0.,0.,0.,1]])
		else:
			self.map_frame_transform = np.array([[1.,0.,0.,8],
												  [0.,1.,0.,9],
												  [0.,0.,1.,-0.182272858645],
												  [0.,0.,0.,1]])
		self.i = 0
		
		self.dist = 0
		self.filename = filename
		self.box_points = self.parse_world(self.filename)
		if self.box_points.shape[0] == 14:
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
			# self.box_list = [0,13]
			self.box_list = [0,1,4,7,12,13]
			# self.box_list=[0]
		if self.box_points.shape[0] == 4:
			self.corners = [["BR","TR"],
							["TR","TL"],
							["BR","BL"],
							["TR","TL"]]
			# self.corners.reverse()
			self.box_list = [0,1,2,3]
		# self.box_list = [0,1]
		self.num_points = len(self.box_list) * 2
		self.avgs = []
		self.errors = []
		self.corner_points = self.calc_corners(self.box_points,self.corners,self.box_list)
		self.filename = 'svd.npy'
		print "BOX",self.box_list[0]
		print self.corners[self.box_list[0]]

	# calculate box corner points given box centroids and directions
	# talk to kevin about this, he knows what it means... :)
	def calc_corners(self,points,directions,box_list):
	    points_list = []
	    for i in range(len(box_list)):
	    	index = box_list[i]
	        # print(temp_point)
	        for j in range(len(directions[index])):
	            temp_point = points[index].copy()
	            temp_point[2] += 0.5
	            if directions[index][j] == "TR":
	                temp_point[0] += 0.5
	                temp_point[1] -= 0.5
	            if directions[index][j] == "BR":
	                temp_point[0] -= 0.5
	                temp_point[1] -= 0.5
	            if directions[index][j] == "TL":
	                temp_point[0] += 0.5
	                temp_point[1] += 0.5
	            if directions[index][j] == "BL":
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

		# print(self.corner_points)

		map_point = np.array([map_x,map_y,map_z,1])
		
		# if self.i % 2 == 0:
		# 	print
		# 	print "BOX",self.box_list[self.i/2]
		# 	print self.corners[self.i/2]
		map_point = map_point[:3]
		if map_point[2] < 0.5:
			print "Invalid point, try again"
			return

		print "Map-Generated Coordinate:", map_point

		gt_point = np.dot(self.map_frame_transform, self.corner_points[self.i])

		gt_point = gt_point[:3]
		print "Ground Truth Coordinate:", gt_point
		diff = map_point - gt_point
		error = np.linalg.norm(map_point - gt_point)	
		print "dx:", diff[0], "m"
		print "dy:", diff[1], "m"
		print "dz:",diff[2],"m"
		print
		print "Error:", error, "m"
		print

		self.i += 1
		self.dist += error

		avg = self.dist / self.i
		self.errors.append(error)

		print "Compounded Average Error:", avg, "m"
		self.avgs.append(avg)
		print("*********")
		
		

		if self.i == self.num_points:
			print("Error test completed")
			array = np.array(self.avgs)
			np.save('/home/kob51/catkin_ws/src/autovalet/autovalet_gazebo/data/'+self.filename,array)
			self.sub.unregister()
			return

		if self.i % 2 == 0:
			print
			box = self.box_list[self.i/2]
			print "BOX", box
			print self.corners[box]
		
		

if __name__ == "__main__":

	rospy.init_node('test')

	filename = sys.argv[1]



	# calc = Error_Calculator('/home/kob51/catkin_ws/src/autovalet/autovalet_gazebo/worlds/slam_val_dumpster.world')
	calc = Error_Calculator(filename)



	rospy.spin()