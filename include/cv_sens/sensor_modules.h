/*
 * sensor_modules.h
 *
 *  Created on: 21-Jan-2016
 *      Author: atulya
 */

#ifndef SENSOR_MODULES_H_
#define SENSOR_MODULES_H_

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <std_msgs/String.h>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#define GROUND_PLANE_POINTS             100                     //camera initial rotation correction parameter

using namespace Eigen;

enum sensor_status_t
{
  UNINITIALIZED,
  ALIVE,
  DIVERGED,
  LOST
};

class SensorIMU
{
public:
  ros::Time last_imu_stamp;
  tf::Vector3 a_b;

  float phi;
  float theta;
  float psi;

public:
  SensorIMU();
};

class SensorSonar
{
public:
  bool flag_sonar_enabled;
  bool flag_sonar_diverged;
  bool flag_sonar_lost;

  ros::Time last_sonar_stamp_device;            //timestamp according to the device clock; use for all fusion
  ros::Time last_sonar_stamp_system;            //timestamp at which data was received on the system; used only for checking sensor timeouts
  float depth;
  float sonar_diverged_time;

public:

  SensorSonar();

  //initialize the SONAR
  void initialize(float z, ros::Time device_stamp, ros::Time system_stamp);

  //checks if raw data is totally out of range
  bool isGlitching(float z);

  //process Sonar data
  void processData(float z, ros::Time device_stamp, ros::Time system_stamp);
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


#endif /* SENSOR_MODULES_H_ */
