#!/bin/bash
#export ROS_MASTER_URI=http://odroid:11311
rosrun rqt_plot rqt_plot /mavros/local_position_ned/position/vector/z /mavros/sim_state/position/vector/z /mavros/sim_state/accel/vector/z /mavros/sim_state/velocity/vector/z
