# autovalet_lane_detection

This package contains our lane detection pipeline. The output of the lane detection will be used projected on the costmap, so that valid goals lying within the lane can be sent to the planner.

## Running the lane detector node in Hardware
```
roslaunch autovalet_lane_detection lane_detector.launch simulation:=false
```

## Running the lane detector node in Simulation

To see the lane detection in action, do the following:

```
roslaunch autovalet_gazebo parking_lot.launch world_file:=slam_val_dumpster.world
```  

```
roslaunch autovalet_lane_detection lane_detector.launch
```

To view the lane as a pointcloud, use the pointcloud2 widget in Rviz and subscribe to the /lane/pointCloud topic

Note: Running lane_detector_sim.launch would also start the goal generator in sim 
