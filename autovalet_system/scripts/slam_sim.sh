#!/bin/bash
roslaunch autovalet_gazebo setup_slam_test.launch world_file:=state_machine.world x:=0 y:=6 > $HOME/slam_log.txt