/*
 * pattern_track.h
 *
 *  Created on: 02-Feb-2016
 *      Author: atulya
 */

#ifndef PATTERN_TRACK_H_
#define PATTERN_TRACK_H_

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include <sensor_msgs/NavSatFix.h>
#include <px_comm/OpticalFlow.h>
#include <mavros_msgs/RCIn.h>
//TODO remove unnecessary topics

#include <vikit/user_input_thread.h>

#define CALIBRATION_INLIER_ERR_THRSH    9.0*M_PI/180.0f
#define CALIBRATION_NUM_VARIABLES       20                      //assuming 10 sec @2Hz
#define CALIBRATION_INLIER_PERCENT      75

class Tracker
{
  //All possible rotation matrices according to the different frames
  tf::Matrix3x3 R_CB_EM;
  tf::Matrix3x3 R_EB_CIB;
  tf::Matrix3x3 R_EE_EB;
  tf::Matrix3x3 R_EE_EM;
  tf::Matrix3x3 R_CB_CIB;
  tf::Matrix3x3 R_EE_EHBF;
  tf::Matrix3x3 R_CB_CIB_ideal;

  cv::Mat inImage;
  aruco::CameraParameters camParam;

  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  vector<aruco::Marker> markers;

  bool cam_info_received;

  double marker_size;
  int marker_id;

  image_transport::ImageTransport it;
  image_transport::Publisher image_pub;

  bool flag_calibration_started;
  bool flag_calibration_image_acquired;
  bool flag_calibration_finished;
  int calibration_count;
  int calibration_inlier_count;


public:

  boost::shared_ptr<vk::UserInputThread> keyboard_input;

  geometry_msgs::Vector3Stamped rpy_imu_msg;
  ros::Publisher rpy_imu_pub;

  geometry_msgs::Vector3Stamped rpy_cam_msg;
  ros::Publisher rpy_cam_pub;

  image_transport::Subscriber image_sub;
  ros::Subscriber cam_info_sub;

  bool flag_new_marker_received;
  float target_height;

  float marker_pos[3];
  float marker_yaw;

public:

  Tracker(ros::NodeHandle nh, float z);
  ~Tracker();

  void processUserAction();

  void initializeRotation();

  void useImageForCalibration();

  void bypassCalibration();

  bool isCalibrationFinished();

  void image_callback(const sensor_msgs::Image::ConstPtr& msg);

  void cam_info_callback(const sensor_msgs::CameraInfo &cam_info);

  void markerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  void publishDebugMessages();

};


#endif /* PATTERN_TRACK_H_ */
