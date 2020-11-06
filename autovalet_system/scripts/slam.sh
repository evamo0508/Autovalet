#!/bin/bash
sim=$1
turn=$2
if [ "$sim" = "true" ]
then
    if [ "$turn" = "right" ]
    then
        roslaunch autovalet_gazebo setup_slam_test.launch world_file:=state_machine.world x:=0 y:=6 1>> $HOME/slam_log.txt 2>&1
    else
        roslaunch autovalet_gazebo setup_slam_test.launch world_file:=state_machine_left.world x:=-6 y:=-9 1>> $HOME/slam_log.txt 2>&1
    fi
else
    roslaunch autovalet_husky integrated_sensor.launch 1>> $HOME/sensor_log.txt 2>&1
    roslaunch autovalet_husky valet_rtab.launch 1>> $HOME/slam_log.txt 2>&1
fi