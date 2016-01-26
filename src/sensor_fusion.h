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
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <px_comm/OpticalFlow.h>
#include <mavros_msgs/RCIn.h>

#include "ekf.h"
#include "sensor_modules.h"

#define ROLLING_FILTER_WINDOW           40
#define IMU_FREQUENCY                   52.5f
#define DEFAULT_HEIGHT                  0.8f
#define SONAR_INITIALIZATION_MAX_HEIGHT 1.2f
#define GRAVITY_MSS                     9.81f

#define MAX_BUFFER_SIZE                 150
#define STD_DEV_THRESHOLD               0.18f
#define DATA_DELAY                      6
#define SONAR_DELTA_READING_MAX         0.6f

#define IMU_LOSS_THRESHOLD              0.5f
#define VISION_BREAKSIGNAL_THRESHOLD    2                       //time to wait before assuming sensor data has been lost
#define SONAR_LOSS_TIME_THRESHOLD       3                       //time to wait before assuming sensor data has been lost

#define DEBUG_WITH_ROSBAG               1                       //if 1 we are checking the working on a rosbag 0 for realtime operation
#define SYSTEM_ID                       6

class RollingQueue
{
  // z queue maintaining last values
  std::vector<double> rolling_q;
  int start, end, size;
  float default_value;
  bool is_full;

public:

  RollingQueue(int length, float default_val);

  //reset the queue with some default initial value
  void reset();

  //insert an element into the queue
  void insert(float z);

  void printQueue();

  //if some predefined number of data points have been inserted the queue is said to be initialized
  inline bool isInitialized() {
    return is_full;
  }

  //finds the median of data and returns true if input - median > threshold
  bool isOutlierWithMedianFilter(float input, float threshold);
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
  bool checkCompletion();

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
  //object for the IMU data
  SensorIMU imu;

  //object for the sonar
  SensorSonar sonar;
  RollingQueue sonar_queue;
  ros::Time last_good_sonar_stamp;

  //object for the vslam
  SensorCV cv_slam;

  struct SensorHealth
  {
    sensor_status_t cv;
    sensor_status_t sonar;
    sensor_status_t imu;
    sensor_status_t baro;
  }sensor_health;

  struct Flags
  {
    bool lambda_initialized;            //initializing the flag variable for the estiation of lambda
    bool ekf_prediction_enabled;        //when it is true ekf prediction is enabled
  }flags;

  //storing the corresponding visual and sonar data in a queue
  Queue cv_sonar_correspondance;

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

  //reset the Sonar
  void resetSonar();

  //The camera of the quad may not be perfectly vertical because of which the ground is not perfectly horizontal
  //This function corrects the rotation such that the map points which are obtained are on a horizontal ground
  void updateVSLAM_MapPointsRotationCorrection(const visualization_msgs::Marker::ConstPtr& msg);

  //update the VSLAM data
  void updateVSLAMData(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  //reset the VSLAM
  void resetVSLAM();

  //checks whether VSLAM is running fine
  bool isVSLAMEstimationHealthy();

  //checks whether Altitude being published is healthy
  bool isAltitudeHealthy();

  //Monitors the health of different sensor modalities
  void monitorSensorHealth(diagnostic_updater::DiagnosticStatusWrapper &stat);

  //publishes different debugging messages
  void publishDebugMessages(ros::Time stamp);

};

#endif /* SENSOR_FUSION_H_ */
