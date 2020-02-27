This is a restructured version of gazebo_simulation. The cmu_parking_lot world contains world description without the husky in it. (TBD: Gound plane not showing up).

The models folder contains the ground plane model.

Launch Files:
1) parking_lot:	
	--> launches modified urdf whih passes the robot model onto parameter server (model available as robot_description parameter).
	-->	opens our world in gazebo
	-->	spawns husky in gazebo with modified ursd (looks up the robot_description parameter on the param server)
	-->	We have our copy of the husky_control launch file. Done because it was launching a state publisher. We have ours in huskyValet_urdf.launch. (TBD: remove state and joint publisher in urdf launch and use standard husky package's husky_control launch file)
	--> Run teleoptwist keyboard with topic remap.

Terminal command:
1) roslaunch autovalet_gazebo parking_lot.launch

This should launch the world, spawn modified husky, run teleop node.

2) rosrun rviz rviz

After opening rviz, add robot model to visualize robot. Change fixed frame to base_footprint. Add pointcloud2 and subscribe to /velodyne_points to see pcl data. Change size to 0.03, color transformer to axiscolor. (TBD: save and call rviz config file with flag)