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

#include <vikit/user_input_thread.h>

#define CALIBRATION_INLIER_ERR_THRSH    9.0*M_PI/180.0f
#define CALIBRATION_NUM_VARIABLES       20                      //assuming 10 sec @2Hz
#define CALIBRATION_INLIER_PERCENT      75

class Pattern_Tracker
{
  //TODO rename all variables according to the ROS naming convention in the next documentation phase
  //All possible rotation matrices according to the different frames
  tf::Matrix3x3 R_CB_EM;
  tf::Matrix3x3 R_EB_CIB;
  tf::Matrix3x3 R_EE_EB;
  tf::Matrix3x3 R_EE_EM;
  tf::Matrix3x3 R_CB_CIB;
  tf::Matrix3x3 R_EE_EHBF;
  tf::Matrix3x3 R_CB_CIB_ideal;

  tf::Vector3 t_em_cb;
  tf::Vector3 relative_target;

  cv::Mat inImage;
  aruco::CameraParameters camParam;

  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  vector<aruco::Marker> markers;

  bool cam_info_received;

  double marker_size_;
  int marker_id_;

  image_transport::ImageTransport it;
  image_transport::Publisher image_pub;

  bool flag_calibration_started;
  bool flag_calibration_image_acquired;
  bool flag_calibration_finished;
  int calibration_count;
  int calibration_inlier_count;

  geometry_msgs::Vector3Stamped rpy_imu_msg;
  ros::Publisher rpy_imu_pub;

  geometry_msgs::Vector3Stamped rpy_cam_msg;
  ros::Publisher rpy_cam_pub;

  geometry_msgs::Vector3Stamped relative_position_msg;
  ros::Publisher relative_position_pub;

  image_transport::Subscriber image_sub;
  ros::Subscriber cam_info_sub;
  ros::Subscriber imu_sub;

public:

  boost::shared_ptr<vk::UserInputThread> keyboard_input;

  bool flag_new_marker_received;
  float target_height;

  tf::Vector3 marker_pos_;
  float marker_yaw;

private:
  // draw XYZ axes around the detected marker in the image
  void draw3dAxis(aruco::Marker m);

  // transform marker pose from camera frame to the HBF frame
  void transformMarkerPoseToHBFFrame(tf::Matrix3x3 marker_rot, tf::Vector3 marker_pos);

  // check if calibration is correct for some number of marker detections
  void updateCalibrationCheck();

  // initialize rotation matrix for the marker assuming quad is kept over it
  void initializeRotation();

  // use the last detected marker image for calibration
  void useImageForCalibration();

  // bypass the calibration step completely, assuming that the camera has been placed vertically
  void bypassCalibration();

  // callback for the image function
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  // callback for the camera info
  void camInfoCallback(const sensor_msgs::CameraInfo &cam_info);

  // callback from IMU
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

public:

  Pattern_Tracker(ros::NodeHandle nh, float target_z,
                  float marker_size, int marker_id);
  ~Pattern_Tracker();

  // check if the calibration step has been completed
  bool isCalibrationFinished() const;

  // process user keyboard action
  void processUserAction();

  // callback for the direct marker from aruco
  void markerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  // publish debug messages
  void publishDebugMessages();

  //returns the camera position as viewed from MarkerFrame
  tf::Vector3 getCameraPosInMarkerFrame() const;

  // obtain waypoint to achieve the desired position
  // note that the target position is defined in the marker frame assuming that it remains horizontal
  // the returned value is in E_HBF frame
  tf::Vector3 getDesiredMovementToReachTarget(tf::Vector3 pos_target) const;

};


#endif /* PATTERN_TRACK_H_ */
