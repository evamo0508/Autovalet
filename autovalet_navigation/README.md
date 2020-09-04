# Navigation Stack

## Running the navigation stack
1. Make sure you have the [TEB Local Planner](http://wiki.ros.org/teb_local_planner) package installed
```
sudo apt-get install ros-kinetic-teb-local-planner
```

2. Launch the Gazebo world and begin mapping
```
roslaunch autovalet_gazebo setup_slam_test.launch
```

3. In another terminal, launch the navigation node
```
roslaunch autovalet_navigation autovalet_nav.launch
```

4. In another terminal, launch the state machine pipeline for easier debugging
```
roslaunch autovalet_navigation pipeline.launch
```

5. In RViz, use the `2d Nav Goal` tool to specify goal poses for the Husky. Make sure you choose goal poses that lie on the map, otherwise the planning will fail. Follow along with the pipeline's output to monitor system progress.