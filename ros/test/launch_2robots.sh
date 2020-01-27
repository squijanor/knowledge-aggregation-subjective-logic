#!/bin/sh

# Set rocore and gazebo ports where the nodes will pub/sub
export ROS_MASTER_URI=http://localhost:11315
export GAZEBO_MASTER_URI=http://localhost:11335

gnome-terminal --tab -- bash -c "roslaunch master.launch gen_seed:=110 use_sl:=true sl_operator:=CBF adaptive_scheduling:=true; exec bash"

#gnome-terminal --tab -- bash -c "rviz -d multi_robot.rviz; exec bash"
