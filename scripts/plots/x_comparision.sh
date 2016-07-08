#!/bin/bash
#export ROS_MASTER_URI=http://odroid:11311
rosrun rqt_plot rqt_plot /mavros/local_position_ned/position/vector/x /mavros/sim_state/position/vector/x /mavros/hil_state/pos_target/vector/x
