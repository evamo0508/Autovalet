#!/bin/bash
sleep 10

logfile=$HOME/slam_log.txt
rm $logfile

roslaunch autovalet_navigation autovalet_nav.launch simulation:=$1 1>> $logfile 2>&1

