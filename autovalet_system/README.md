# AutoValet System

This package contains the state machine for the full AutoValet pipeline.

## Setup
Make sure all of the files in the `/scripts` directory are executable
```
chmod +x ~/catkin_autovalet/src/autovalet/autovalet_system/scripts/*
```

## Launching the system
The system will open up a new terminal displaying helpful system status messages. This is the only terminal that you need to look at when runinng the system.

**Running in Gazebo**
```
roslaunch autovalet_system state_machine.launch simulation:=true
```

**Running on the Husky**
```
roslaunch autovalet_system state_machine.launch simulation:=false
```
