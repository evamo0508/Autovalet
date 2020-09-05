# autovalet_husky

This package contains launch files for running the physical robot on hardware.

## Running the system (hardware)

1. To run the Husky with all sensors and joystick teleop:

```
roslaunch autovalet_husky integrated_sensor.launch

```

2. Then run the ArUco marker detection:
```
roslaunch autovalet_husky aruco_hardware.launch

```

3. Run the SLAM stack:

```
roslaunch autovalet_husky valet_rtab.launch 

```

3. Run the navigation stack:

```
roslaunch autovalet_navigation autovalet_nav.launch simulation:=false

```

4. Run the script that finds the empty parking spot and navigates to it:

```
cd ~/catkin_ws_av/src/autovalet/autovalet_husky/scripts
python find_parking_spot.py

```

## Running the system (simulation)

1. To launch custom Gazebo world:

```
roslaunch autovalet_gazebo parking_lot.launch world_file:=aruco_vert_dumpster.world

```

2. Then run the ArUco marker detection:
```
roslaunch autovalet_gazebo aruco_sim.launch 

```

3. Run the SLAM stack:

```
roslaunch autovalet_gazebo valet_rtab_sim.launch 

```

3. Run the navigation stack:

```
roslaunch autovalet_navigation autovalet_nav.launch simulation:=true

```

4. Run the script that finds the empty parking spot and navigates to it:

```
cd ~/catkin_ws_av/src/autovalet/autovalet_husky/scripts
python find_parking_spot.py

```
