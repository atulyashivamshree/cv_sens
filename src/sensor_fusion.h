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
#define IMU_FREQUENCY 52.5
#define DEFAULT_HEIGHT 0.8
#define GRAVITY_MSS 9.81

#define MAX_BUFFER_SIZE 150
#define STD_DEV_THRESHOLD 0.18
#define DATA_DELAY 6
#define SONAR_DELTA_READING_MAX 0.4

#define GROUND_PLANE_POINTS 100

//current orientation
extern double phi, theta, psi;

//initializing using the initial values
extern bool flag_sonar_enabled;
extern bool flag_lamda_initialized;

//timestamp of the last visual sensor
extern ros::Time last_cv_stamp;

//storing the corresponding visual and sonar data in a queue
extern Queue cv_sonar_correspondance;

//position x,y,z acquired from vslam in the inertial frame
extern tf::Vector3 position_vgnd;

//rotation matrices
extern tf::Matrix3x3 R_HBF_to_GND;
extern tf::Matrix3x3 R_b_to_HBF;
extern tf::Matrix3x3 R_b_to_GND;
extern tf::Matrix3x3 R_b_to_VGND;
extern tf::Matrix3x3 R_vb_vgnd_t;
extern tf::Matrix3x3 R_calib;
extern double phi_v, theta_v, psi_v;

//acceleration in the horizontal body fixed frame
extern tf::Vector3 accel_hbf;

//position as sensed in vslam frame with respect to an inertial frame
extern geometry_msgs::Vector3Stamped pos_msg_vgnd;
extern ros::Publisher pos_pub_vgnd;

//position as sensed in vslam body frame
extern geometry_msgs::Vector3Stamped pos_msg_vb;
extern ros::Publisher pos_pub_vb;

//acceleration as sensed in horizontal body frame
extern geometry_msgs::Vector3Stamped acc_msg_hbf;
extern ros::Publisher acc_pub_hbf;

//estimated state z_hat and z_hat_dot
extern geometry_msgs::Vector3Stamped z_hat_msg;
extern ros::Publisher z_hat_pub;

//publishing the current position and velocity in VGND frame
extern geometry_msgs::PoseStamped cv_pose_msg;
extern ros::Publisher cv_pose_pub;

//publishing svo_remote key
extern std_msgs::String key_msg;
extern ros::Publisher svo_remote_key_pub;

//converting and publishing the waypoint from ned-i to ned-b frame
extern geometry_msgs::Vector3 waypoint_msg_nedb;
extern ros::Publisher waypoint_pub;

extern visualization_msgs::Marker points_msg;
extern ros::Publisher points_pub;

// bool for setting the home
extern bool SVO_INITIATED;
extern bool SVO_RESET_SENT;
extern bool FLAG_CAMERA_CALIBRATED;

// kalman observer for x and y
extern IMU_CV_EKF ekf_z;
extern geometry_msgs::Vector3Stamped z_static_states_msg;
extern ros::Publisher z_static_states_pub;

//stores the initial ground points which are used for calibrating the camera
extern std::vector<Vector3f> ground_points;
extern int ground_points_index;
extern Vector3f normal;
extern float depth;

// z queue maintaining last values
extern std::vector<double> rolling_q;
extern int start, end;

void resetCV();

void calibrateCamera();

bool isOutlier(double z);

#endif /* SENSOR_FUSION_H_ */
