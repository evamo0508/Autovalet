#!/bin/bash
sim=$1
if [ "$sim" = "true" ] 
then
    roslaunch autovalet_gazebo setup_slam_test.launch world_file:=state_machine.world x:=0 y:=6 1>> $HOME/slam_log.txt 2>&1
else
    roslaunch autovalet_husky integrated_sensor.launch 1>> $HOME/sensor_log.txt 2>&1
    roslaunch autovalet_husky valet_rtab.launch 1>> $HOME/slam_log.txt 2>&1
fi