#!/bin/bash
#export ROS_MASTER_URI=http://odroid:11311
rosrun rqt_plot rqt_plot /mavros/sim_state/accel/vector/z /mavros/hil_state/accel_target/vector/z
