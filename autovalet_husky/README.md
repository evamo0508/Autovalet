# autovalet_husky

This package contains launch files for running the physical robot on hardware.

## Running the system (hardware)

1. To run the Husky with all sensors and joystick teleop:

```
roslaunch autovalet\_husky integrated_sensor.launch

```

2. Then run the ArUco marker detection:
```
roslaunch autovalet\_husky aruco_hardware.launch

```

3. Run the SLAM stack:

```
roslaunch autovalet\_husky valet_rtab.launch 

```

3. Run the navigation stack:

```
roslaunch autovalet\_navigation autovalet_nav.launch simulation:=false

```

4. Run the script that finds the empty parking spot and navigates to it:

```
cd ~/catkin_ws_av/src/autovalet/autovalet_husky/scripts
python find\_parking_spot.py

```

## Running the system (simulation)

1. To launch custom Gazebo world:

```
roslaunch autovalet\_gazebo parking\_lot.launch world\_file:=aruco\_vert_dumpster.world

```

2. Then run the ArUco marker detection:
```
roslaunch autovalet\_gazebo aruco_sim.launch 

```

3. Run the SLAM stack:

```
roslaunch autovalet\_gazebo valet\_rtab_sim.launch 

```

3. Run the navigation stack:

```
roslaunch autovalet\_navigation autovalet_nav.launch simulation:=true

```

4. Run the script that finds the empty parking spot and navigates to it:

```
cd ~/catkin_ws_av/src/autovalet/autovalet_husky/scripts
python find_parking_spot.py

```
