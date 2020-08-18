# autovalet_lane_detection

This package contains our lane detection pipeline. The output of the lane detection will be used projected on the costmap, so that valid goals lying within the lane can be sent to the planner.

## Running the lane detector node in Hardware
```
roslaunch autovalet_lane_detection lane_detector.launch
```

## Running the lane detector node in Simulation

```
roslaunch autovalet_lane_detection lane_detector_sim.launch
```
