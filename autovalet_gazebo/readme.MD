# autovalet_gazebo

This package handles our custom simulated world, modeled after the CUC garage on CMU's campus. This package also handles the mapping capability (`valet_rtab_sim.launch`), which is an implementation of the RTABMap algorithm using an Intel Realsense D435 and a Velodyne VLP-16.

## Running the baseline world

To run the baseline world and have keyboard teleop control of the Husky:

```
roslaunch autovalet_gazebo parking_lot.launch
```

## Running the world with RTABMap

To run the world while also running the mapping system:
```
roslaunch autovalet_gazebo setup_slam_test.launch
```

Note: by default, the above file uses a USB joystick to control the Husky in the world. Change the `joystick` parameter to `false` in order to use keyboard teleop control

