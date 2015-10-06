/*
 * gps_position.cpp
 *
 *  Created on: 29-May-2015
 *      Author: atulya
 */
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include <px_comm/OpticalFlow.h>
#include "nav_gps/imu_acc.h"
#include "nav_gps/state_order6.h"

#include "ekf.h"

#define ROLLING_FILTER_WINDOW 40
#define IMU_FREQUENCY 50.0
#define DEFAULT_HEIGHT -0.8

//current orientation
double phi, theta, psi;

//position x,y,z acquired from vslam in the inertial frame
tf::Vector3 position_vgnd;

//rotation matrices
tf::Matrix3x3 R_HBF_to_GND;
tf::Matrix3x3 R_b_to_HBF;
tf::Matrix3x3 R_b_to_GND;
tf::Matrix3x3 R_b_to_VGND;
tf::Matrix3x3 R_vb_vgnd_t;
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

// bool for setting the home
bool SVO_INITIATED = false;
bool SVO_RESET_SENT = false;

// kalman observer for x and y
IMU_CV_EKF ekf_z(1/IMU_FREQUENCY);
geometry_msgs::Vector3Stamped z_static_states_msg;
ros::Publisher z_static_states_pub;

// z queue maintaining last values
std::vector<double> rolling_q(ROLLING_FILTER_WINDOW, DEFAULT_HEIGHT);
int start, end;

void statsCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  int chnl6 = msg->vector.z;
  if(chnl6 > 2048 && SVO_INITIATED == false && SVO_RESET_SENT == false)
  {
    key_msg.data = "r";
    svo_remote_key_pub.publish(key_msg);
    SVO_RESET_SENT = true;

  }
  if(chnl6 > 2048 && SVO_INITIATED == false && SVO_RESET_SENT == true)
  {
    key_msg.data = "s";
    svo_remote_key_pub.publish(key_msg);

    SVO_INITIATED = true;
  }
  if(chnl6 < 2048)
  {
    SVO_RESET_SENT = false;
    SVO_INITIATED = false;
  }
}

void waypointNEDICallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  tf::Vector3 wp_nedi(msg->x, msg->y, msg->z);
  //converting from nedi to nedb
  tf::Vector3 wp_nedb =  R_vb_vgnd_t*wp_nedi;

  tf::vector3TFToMsg(wp_nedb, waypoint_msg_nedb);
  waypoint_pub.publish(waypoint_msg_nedb);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  geometry_msgs::Quaternion q_ = msg->orientation;
  tf::Quaternion q;
  tf::quaternionMsgToTF(q_, q);

  R_b_to_GND.setRotation(q);

  double psi_temp;

  R_b_to_GND.getRPY(phi, theta, psi_temp);

//  R_HBF_to_GND.setRPY(0, 0, psi);
  R_b_to_HBF.setRPY(phi, theta, 0);

  tf::Vector3 a_b(msg->linear_acceleration.x,
                  msg->linear_acceleration.y,
                  msg->linear_acceleration.z);

  tf::Vector3 g(0,0,9.81);	//the gravgndty vector

  accel_hbf = R_b_to_HBF.transpose()*a_b + g;           //acceleration as obtained in the horizontal body frame and in NED coordinate system

  accel_hbf.setZ(-accel_hbf.getZ());                    //inverting z acceleration to deal with altitude which is positive upwards

  if(ekf_z.x_hat_kplus1_kplus1(0,0) > 0.3)             // the absolute lower limit possible for ultrasonic sensors
    ekf_z.prediction(accel_hbf.getZ());

  tf::vector3TFToMsg(accel_hbf, acc_msg_hbf.vector);
  acc_msg_hbf.header.stamp = msg->header.stamp;

  acc_pub_hbf.publish(acc_msg_hbf);
}

void vslamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  position_vgnd.setX(msg->pose.pose.position.x);
  position_vgnd.setY(msg->pose.pose.position.x);
  position_vgnd.setZ(msg->pose.pose.position.x);

  //position in the vslam inertial frame used for debugging data
  pos_msg_vgnd.header.stamp = msg->header.stamp;
  pos_msg_vgnd.vector.x = position_vgnd.getX();
  pos_msg_vgnd.vector.y = position_vgnd.getY();
  pos_msg_vgnd.vector.z = position_vgnd.getZ();

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  R_b_to_VGND.setRotation(q);
  R_b_to_VGND.getRPY(phi_v, theta_v, psi_v);

  std::cout<<"psi_v i "<<psi_v<<"\n";
  R_HBF_to_GND.setRPY(0,0,psi_v);

  //update the vslam algorithm according to the inertial matrix
  ekf_z.updateVSLAM(position_vgnd.getZ());

  //publishing pos and velocity in visual GND frame(which is at some psi_0 with respect to the GND frame) calculated to the HLP
  cv_pose_msg.header.stamp = msg->header.stamp;
  cv_pose_msg.pose.position.x = ekf_z.x_hat_kplus1_kplus1(3,0)*position_vgnd.getX();
  cv_pose_msg.pose.position.y = ekf_z.x_hat_kplus1_kplus1(3,0)*position_vgnd.getY();
  cv_pose_msg.pose.position.z = ekf_z.x_hat_kplus1_kplus1(0,0);
  tf::Quaternion q_hbf_to__vgnd;
  R_HBF_to_GND.getRotation(q_hbf_to__vgnd);
  tf::quaternionTFToMsg(q_hbf_to__vgnd, cv_pose_msg.pose.orientation);

  z_static_states_msg.header.stamp = msg->header.stamp;
  z_static_states_msg.vector.x = ekf_z.x_hat_kplus1_kplus1(2,0);
  z_static_states_msg.vector.y = ekf_z.x_hat_kplus1_kplus1(3,0);
  z_static_states_msg.vector.z = ekf_z.x_hat_kplus1_kplus1(4,0);

  pos_pub_vgnd.publish(pos_msg_vgnd);
  cv_pose_pub.publish(cv_pose_msg);
  z_static_states_pub.publish(z_static_states_msg);

//  ROS_INFO("psi is %f", psi);
}

bool isOutlier(double z)
{
  rolling_q[ROLLING_FILTER_WINDOW-1] = z;

  std::vector<double> sorted_q = rolling_q;
  std::sort(sorted_q.begin(), sorted_q.end());

  for(int i = 0; i<rolling_q.size()-1; i++)
    rolling_q[i] = rolling_q[i+1];

  double median = sorted_q[ROLLING_FILTER_WINDOW/2];

//  std::cout<<"front is "<<sorted_q[0]<<"end-2 is "<<sorted_q[ROLLING_FILTER_WINDOW-3]<<"median is" <<median<<"z is "<<z<<"diff is "<<abs(z-median)<<"\n";

  if(fabs(z-median) > 0.4)
    return true;
  else
    return false;

}

void px4flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
  double z = msg->ground_distance;

  if(isOutlier(z))
  {
    rolling_q[ROLLING_FILTER_WINDOW-2] = ekf_z.x_hat_kplus1_kplus1(0,0);
//    std::cout<<"outlier detected";
  }
  else
  {
    rolling_q[ROLLING_FILTER_WINDOW-2] = z;
    ekf_z.updateUltrasonic(z);
  }

  z_hat_msg.header.stamp = msg->header.stamp;
  z_hat_msg.vector.x = ekf_z.x_hat_kplus1_kplus1(0,0);
  z_hat_msg.vector.y = ekf_z.x_hat_kplus1_kplus1(3,0)*position_vgnd.getZ() + ekf_z.x_hat_kplus1_kplus1(4,0) ;
//  z_hat_msg.vector.z = ekf_z.x_hat_kplus1_kplus1(2,0);
  z_hat_msg.vector.z = z;

  z_hat_pub.publish(z_hat_msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ekf_vslam_fusion");
  ros::NodeHandle n;

  R_vb_vgnd_t.setRPY(0,0,0);

  ros::Subscriber vslam_sub = n.subscribe("svo/pose", 10, vslamCallback);
  ros::Subscriber stats_sub = n.subscribe("stats", 100, statsCallback);
  ros::Subscriber rpy_sub = n.subscribe("imu/data_raw", 100, imuCallback);
  ros::Subscriber px4flow_sub = n.subscribe("px4flow/opt_flow", 10, px4flowCallback);
  ros::Subscriber waypoint_sub = n.subscribe("waypoint_ned_i", 5, waypointNEDICallback);

  pos_pub_vgnd = n.advertise<geometry_msgs::Vector3Stamped>("cv_pose", 100);
  acc_pub_hbf = n.advertise<geometry_msgs::Vector3Stamped>("sens_acc_hbf", 100);
  z_hat_pub = n.advertise<geometry_msgs::Vector3Stamped>("z_hat_ekf", 100);
  cv_pose_pub = n.advertise<geometry_msgs::PoseStamped>("sens_pos", 100);
  svo_remote_key_pub = n.advertise<std_msgs::String>("svo/remote_key", 5);
  z_static_states_pub = n.advertise<geometry_msgs::Vector3Stamped>("z_static_states", 100);
  waypoint_pub = n.advertise<geometry_msgs::Vector3>("waypoint", 5);

  ros::spin();
}
