***
PARKING SPOT DETECTION USING ARUCO MARKERS
***

This package consists of gazebo models (aruco\_gazebo by https://github.com/joselusl/aruco_gazebo.git) and the detection scripts (aruco\_ros by https://github.com/pal-robotics/aruco_ros.git) for ArUco markers detection. 
We utilized model number 7 (aruco\_visual\_marker\_7) for our purpose and inserted it into our custom world models (autovalet/autovalet\_gazebo/models/). The marker pose and marker size in the model.sdf file was modified to suit our requirements. This model was then imported into a new modified world : aruco\_vert\_dumpster.world which is launched using parking\_lot.launch.

$ roslaunch autovalet_gazebo parking_lot.launch 

For detection in simulation, we only need to run the single.launch file with the correct arguments. This file has been renamed to aruco\_sim.launch and moved to autovalet/autovalet\_gazebo/launch/. A number of parameters were modified here based on the type of camera and the camera frame being used.

$ roslaunch autovalet_gazebo aruco_sim.launch markerId:=7  markerSize:=0.5 ref_frame:=/base_link

When both files are launched, go to RViz and change the image topic to /aruco\_single/result. As you teleoperate the robot in the terminal and move closer to the tag, you will notice a frame that'll show in the image when the robot is close enough to detect the tag/marker. Echo the pose to see the detected values.

$ rostopic echo /aruco_single/pose


For detection in hardware, we again run the single.launch file, now renamed as aruco\_hardware.launch and moved to autovalet/autovalet\_husky/launch/. 
$ roslaunch autovalet_husky integrated_sensor.launch 
$ roslaunch autovalet_husky aruco_hardware.launch markerId:=26  markerSize:=0.165 ref_frame:=/base_link

The validation script that shows the translational error between estimated and ground truth position of the arUco marker in sim is autovalet/autovalet_gazebo/scripts/validate_arucotags.py.
