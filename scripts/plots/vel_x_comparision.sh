#!/bin/bash
#export ROS_MASTER_URI=http://odroid:11311
rosrun rqt_plot rqt_plot /mavros/local_position_ned/position/vector/x /mavros/sim_state/velocity/vector/x /mavros/hil_state/vel_target/vector/x /mavros/hil_state/vel_desired/vector/x
