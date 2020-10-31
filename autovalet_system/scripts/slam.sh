#!/bin/bash
sim=$1
turn=$2
park=$3

if [ "$sim" = "true" ]
then
    if [ "$turn" = "right" ]
    then
        if [ "$park" = "right" ]
        then
            # right turns, right park
            roslaunch autovalet_gazebo setup_slam_test.launch world_file:=state_machine_rturns_rpark.world x:=0 y:=6 1>> $HOME/slam_log.txt 2>&1
        else
            # right turns, left park
            roslaunch autovalet_gazebo setup_slam_test.launch world_file:=state_machine_rturns_lpark.world x:=0 y:=6 1>> $HOME/slam_log.txt 2>&1
        fi
    else
        if [ "$park" = "right" ]
        then
            # left turns, right park
            roslaunch autovalet_gazebo setup_slam_test.launch world_file:=state_machine_lturns_rpark.world x:=-6 y:=-9 1>> $HOME/slam_log.txt 2>&1
        else
            # left turns, left park
            roslaunch autovalet_gazebo setup_slam_test.launch world_file:=state_machine_lturns_lpark.world x:=-6 y:=-9 1>> $HOME/slam_log.txt 2>&1
        fi
    fi
else
    roslaunch autovalet_husky integrated_sensor.launch 1>> $HOME/sensor_log.txt 2>&1
    roslaunch autovalet_husky valet_rtab.launch 1>> $HOME/slam_log.txt 2>&1
fi