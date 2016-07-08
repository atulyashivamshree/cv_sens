#!/bin/bash
#export ROS_MASTER_URI=http://odroid:11311
rosrun rqt_plot rqt_plot /mavros/imu/attitude/vector/x /mavros/sim_state/attitude/vector/x
