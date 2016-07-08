#!/bin/bash
#export ROS_MASTER_URI=http://odroid:11311
rosrun rqt_plot rqt_plot /mavros/sim_state/velocity/vector/z /mavros/hil_state/vel_desired/vector/z /mavros/hil_state/vel_target/vector/z
