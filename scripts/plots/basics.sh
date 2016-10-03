#!/bin/bash
rosrun rqt_plot rqt_plot /mavros/local_position_ned/position/vector/z /mavros/sim_state/position/vector/z /mavros/hil_state/pos_target/vector/z &
rosrun image_view image_view image:=/svo/image &
rosrun rqt_reconfigure rqt_reconfigure &
