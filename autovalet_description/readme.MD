# autovalet_description

This is the autovalet husky URDF description package. We have a modified version of the husky with a LiDAR and 2 d435s on it. The huskyValet_urdf launch file calls xacro parser onto the modified urdf passed to it. Also, it runs robot_state publisher and joint_state publisher for standalone vizualization purposes.

Notes:
The VLP-16 xacro in accessoried folder has a tag called plugins (ctrl+f plugins). In that are paths to source few .so files which establish data transfer between gazebo and ros for the vlp simulated sensor. For those .so files to exist you need to have the velodnye_sensor package which contains the corresponding .cpp files to generate those .so files. Hence there's an extra velodyne simulator package in the autovalet system.
