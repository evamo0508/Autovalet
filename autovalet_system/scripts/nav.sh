#!/bin/bash
sleep 10
roslaunch autovalet_navigation autovalet_nav.launch simulation:=$1 1>> $HOME/nav_log.txt 2>&1

