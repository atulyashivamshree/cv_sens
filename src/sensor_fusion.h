/*
 * sensor_fusion.h
 *
 *  Created on: 18-Nov-2015
 *      Author: atulya
 */

#ifndef SENSOR_FUSION_H_
#define SENSOR_FUSION_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include <visualization_msgs/Marker.h>

#include <px_comm/OpticalFlow.h>
#include <mavros_msgs/RCIn.h>

#include "ekf.h"

#define ROLLING_FILTER_WINDOW           40
#define IMU_FREQUENCY                   52.5
#define DEFAULT_HEIGHT                  0.8
#define GRAVITY_MSS                     9.81

#define MAX_BUFFER_SIZE                 150
#define STD_DEV_THRESHOLD               0.18
#define DATA_DELAY                      6
#define SONAR_DELTA_READING_MAX         0.4

#define VISION_BREAKSIGNAL_THRESHOLD    2                       //time to wait before assuming sensor data has been lost
#define SONAR_LOSS_TIME_THRESHOLD       3                       //time to wait before assuming sensor data has been lost

#define GROUND_PLANE_POINTS             100                     //camera initial rotation correction parameter
#define DEBUG_WITH_ROSBAG               1                       //if 1 we are checking the working on a rosbag 0 for realtime operation

enum sensor_status_t
{
  UNINITIALIZED,
  HEALTHY,
  DIVERGED,
  LOST
};

class SensorSonar
{
public:
  bool flag_sonar_enabled;
  bool flag_sonar_initialized;
  bool flag_sonar_diverged;
  bool flag_sonar_lost;

  ros::Time last_sonar_stamp;
  float depth;
  float sonar_diverged_time;

public:

  SensorSonar();

  //initialize the SONAR
  void initialize(float z, ros::Time stamp);

  //checks if raw data is totally out of range
  bool isGlitching(float z);

  //process Sonar data
  void processData(float z, ros::Time stamp);
};

class SensorCV
{
public:
  // bool for setting the home
  bool flag_VSLAM_initiated;
  bool flag_VSLAM_reset_sent;
  bool flag_camera_rotation_corrected;
  bool flag_VSLAM_lost;

  //timestamp of the last vision sensor
  ros::Time last_cv_stamp;

  //position x,y,z acquired from vslam in the inertial frame
  tf::Vector3 position_vgnd;
  double phi_v, theta_v, psi_v;

  //stores the initial ground points which are used for calibrating the camera
  std::vector<Vector3f> ground_points;
  int ground_points_index;
  Vector3f normal;
  float depth;

  //Rotation matrix for correcting the map points such that the mapped points are horizontal
  tf::Matrix3x3 R_calib;

public:
  SensorCV();

  //correct for the camera rotation with respect to the body
  void correctCameraRotation();

  //initialize camera data
  void initializeVSLAM();

  //process Vision Data
  void processVSLAMData();

  //reset the camera sensor
  void resetCV();
};

class Queue
{
  std::vector<float> bufferX;
  std::vector<float> bufferY;
  float var_sum;
  float sum;
  float std_dev_threshold;
  int max_buffer_size;
  int data_delay;               //if val is6 =>6*25ms = 150ms assuming sonar data comes after a delay of 150ms after vision data and sonar data is coming in at 40Hz

public:
  float mean;
  float std_dev;

public:
  //constructor for the queue
  Queue(float threshold, int delay_count, int max_buffer);

  //inserts elements into the queue
  void insertElement(float valx, float valy);

  //checks if the variance of the data meets a certain minimum threshold
  bool checkData();

  //returns the scale and constant of the vision estimate
  void calculateScale(float &scale, float &z_0);

  //clears the buffers and initializes the scale and bias to zero;
  void clear();

};

class SensorFusion
{
public:
  //position fused
  tf::Vector3 position;

  //current orientation
  double phi, theta, psi;

private:
  //object for the sonar
  SensorSonar sonar;

  //object for the vslam
  SensorCV cv_slam;

  struct SensorHealth
  {
    sensor_status_t cv;
    sensor_status_t sonar;
    sensor_status_t imu;
    sensor_status_t baro;
  }sensor_health;

  //initializing the flag variable for the estiation of lambda
  bool flag_lamda_initialized;

  //storing the corresponding visual and sonar data in a queue
  Queue cv_sonar_correspondance;
  // z queue maintaining last values
  std::vector<double> rolling_q;
  int start, end;

  // kalman observer for estimating scale lambda from z
  IMU_CV_EKF ekf_z;

  tf::Matrix3x3 R_HBF_to_GND;
  tf::Matrix3x3 R_b_to_HBF;
  tf::Matrix3x3 R_b_to_GND;
  tf::Matrix3x3 R_b_to_VGND;

  //acceleration in the horizontal body fixed frame
  tf::Vector3 accel_hbf;

public:
  //the following are messages published by this node

 //msessages and publishers for publishing topics
  geometry_msgs::Vector3Stamped z_static_states_msg;
  ros::Publisher z_static_states_pub;

  //position as sensed in vslam frame with respect to an inertial frame
  geometry_msgs::Vector3Stamped pos_msg_vgnd;
  ros::Publisher pos_pub_vgnd;

  //acceleration as sensed in horizontal body frame
  geometry_msgs::Vector3Stamped acc_msg_hbf;
  ros::Publisher acc_pub_hbf;

  //estimated state z_hat and z_hat_dot
  geometry_msgs::Vector3Stamped z_hat_msg;
  ros::Publisher z_hat_pub;

  //publishing the current position and velocity in VGND frame
  geometry_msgs::PoseStamped cv_pose_msg;
  ros::Publisher cv_pose_pub;

  visualization_msgs::Marker map_points_msg;
  ros::Publisher map_points_pub;

private:
  //checks if the sonar data is an outlier
  bool isSonarDataOutlier(double z);

  //initialize sonar based on previous measurements
  void initializeSonar(double z, ros::Time stamp);

public:

  SensorFusion();

  //update based on imu data
  void updateIMUData(const sensor_msgs::Imu::ConstPtr& msg);

  //update the sonar based measurements
  void updateSonarData(double z, ros::Time stamp);

  //update the VSLAM data
  void updateVSLAMData(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  //checks whether VSLAM is running fine
  bool isVSLAMEstimationHealthy();

  //reset the VSLAM
  void resetVSLAM();

  //The camera of the quad may not be perfectly vertical because of which the ground is not perfectly horizontal
  //This function corrects the rotation such that the map points which are obtained are on a horizontal ground
  void updateVSLAM_MapPointsRotationCorrection(const visualization_msgs::Marker::ConstPtr& msg);

  //Monitors the health of different sensor modalities
  void monitorSensorHealth();

  //publishes different debugging messages
  void publishDebugMessages(ros::Time stamp);

};

#endif /* SENSOR_FUSION_H_ */
