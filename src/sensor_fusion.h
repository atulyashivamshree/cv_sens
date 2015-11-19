/*
 * sensor_fusion.h
 *
 *  Created on: 18-Nov-2015
 *      Author: atulya
 */

#ifndef SENSOR_FUSION_H_
#define SENSOR_FUSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <px_comm/OpticalFlow.h>
#include <mavros_msgs/RCIn.h>

#include "ekf.h"

#define ROLLING_FILTER_WINDOW 40
#define IMU_FREQUENCY 215.0
#define DEFAULT_HEIGHT 0.8
#define GRAVITY_MSS 9.81

#define MAX_BUFFER_SIZE 150
#define STD_DEV_THRESHOLD 0.18
#define DATA_DELAY 6

#define GROUND_PLANE_POINTS 100

//current orientation
double phi, theta, psi;

//initializing using the initial values
bool flag_sonar_enabled;
bool flag_lamda_initialized;

//timestamp of the last visual sensor
ros::Time last_cv_stamp;

//storing the corresponding visual and sonar data in a queue
Queue cv_sonar_correspondance(STD_DEV_THRESHOLD, DATA_DELAY, MAX_BUFFER_SIZE);

//position x,y,z acquired from vslam in the inertial frame
tf::Vector3 position_vgnd;

//rotation matrices
tf::Matrix3x3 R_HBF_to_GND;
tf::Matrix3x3 R_b_to_HBF;
tf::Matrix3x3 R_b_to_GND;
tf::Matrix3x3 R_b_to_VGND;
tf::Matrix3x3 R_vb_vgnd_t;
tf::Matrix3x3 R_calib;
double phi_v, theta_v, psi_v;

//acceleration in the horizontal body fixed frame
tf::Vector3 accel_hbf;

//position as sensed in vslam frame with respect to an inertial frame
geometry_msgs::Vector3Stamped pos_msg_vgnd;
ros::Publisher pos_pub_vgnd;

//position as sensed in vslam body frame
geometry_msgs::Vector3Stamped pos_msg_vb;
ros::Publisher pos_pub_vb;

//acceleration as sensed in horizontal body frame
geometry_msgs::Vector3Stamped acc_msg_hbf;
ros::Publisher acc_pub_hbf;

//estimated state z_hat and z_hat_dot
geometry_msgs::Vector3Stamped z_hat_msg;
ros::Publisher z_hat_pub;

//publishing the current position and velocity in VGND frame
geometry_msgs::PoseStamped cv_pose_msg;
ros::Publisher cv_pose_pub;

//publishing svo_remote key
std_msgs::String key_msg;
ros::Publisher svo_remote_key_pub;

//converting and publishing the waypoint from ned-i to ned-b frame
geometry_msgs::Vector3 waypoint_msg_nedb;
ros::Publisher waypoint_pub;

visualization_msgs::Marker points_msg;
ros::Publisher points_pub;

// bool for setting the home
bool SVO_INITIATED = false;
bool SVO_RESET_SENT = false;
bool FLAG_CAMERA_CALIBRATED = false;

// kalman observer for x and y
IMU_CV_EKF ekf_z(1/IMU_FREQUENCY);
geometry_msgs::Vector3Stamped z_static_states_msg;
ros::Publisher z_static_states_pub;

//stores the initial ground points which are used for calibrating the camera
std::vector<Vector3f> ground_points(GROUND_PLANE_POINTS);
int ground_points_index;
Vector3f normal;
float depth;

// z queue maintaining last values
std::vector<double> rolling_q(ROLLING_FILTER_WINDOW, DEFAULT_HEIGHT);
int start, end;


#endif /* SENSOR_FUSION_H_ */
