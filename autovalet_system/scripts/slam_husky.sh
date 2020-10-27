#!/bin/bash
roslaunch autovalet_husky integrated_sensor.launch > $HOME/sensor_log.txt
roslaunch autovalet_husky valet_rtab.launch > $HOME/slam_log.txt