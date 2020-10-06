import os
import time
import subprocess

# os.system("roscore")
# os.system("gnome-terminal -x rosparam set use_sim_time true")
# os.system("gnome-terminal -x roslaunch autovalet_husky valet_rtab.launch")
# os.system("gnome-terminal -x rosbag play --clock dummy_new_test_2020-10-02-19-13-26.bag")

proc = subprocess.Popen(["roscore"], shell=True,
             stdin=None, stdout=None, stderr=None, close_fds=True)

procq = subprocess.Popen(["roslaunch autovalet_husky valet_rtab.launch"], shell=True,
             stdin=None, stdout=None, stderr=None, close_fds=True)