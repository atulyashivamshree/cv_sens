/*
 * gps_position.cpp
 *
 *  Created on: 29-May-2015
 *      Author: atulya
 */

#include "cv_sens/sensor_fusion.h"

//----------INITIAL CONFIGURATION---------------
#define KNOB_CHANNEL 6
#define CHANNEL_MAX 2157
#define CHANNEL_MIN 840
//==============================================


//--------------Variables for sending resets to SVO---------
//All the channel variables used for sending reset and other signals
//#define CHANNEL_MID (CHANNEL_MAX + CHANNEL_MIN)/2
#define CHANNEL_MID 1600
float chnl_switching = 0;
float chnl_original;
float chnl_switching_cutoff_freq = 0.6;
float chnl_dt = 1/35.0f;

//publishing svo_remote key
std_msgs::String key_msg;
ros::Publisher svo_remote_key_pub;

bool SVO_INITIATED = false;
bool SVO_RESET_SENT = false;

//==========================================================

//Main declaration of the sensor fusion object
SensorFusion sensor_fusion;

//This function is the final link that sends current position estimates and yaw estimate to the flight control unit
/**
 * @brief sends the current position and yaw estimate in NED frame to the flight controller
 * @param stamp current timestamp obtained from ROS
 * @param x coordinate in metres
 * @param y coordinate in metres
 * @param z coordinate in metres
 * @param yaw yaw angle in radians
 * @param flag_cv_active if true indicates that XYZ estimates are active, if false
 *             indicates that computer vision estimates for XY are inactive and only
 *             Z coordinates from sonar are good
 */
void publishSensorPos(ros::Time stamp, float x, float y, float z,
                      float yaw, bool flag_cv_active)
{
  tf::Matrix3x3 R;

  if(flag_cv_active)
    R.setRPY(1.0f, 0.0f, yaw);              //Hard coded similar thing on HLP implyinng CV is active
  else
    R.setRPY(0.0f, 0.0f, yaw);              //Hard coded similar thing on HLP implying CV in inactive

  tf::Quaternion q;
  R.getRotation(q);

  // copy the position and orientation to the pose message
  geometry_msgs::PoseStamped pose_msg;
  tf::quaternionTFToMsg(q, pose_msg.pose.orientation);
  pose_msg.header.stamp = stamp;
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;

  sensor_fusion.cv_pose_msg = pose_msg;

  sensor_fusion.cv_pose_pub.publish(sensor_fusion.cv_pose_msg);
}


//Use this if you want to take some action on the basis of some input from a channel oon the transmitter
void channelCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
  float alpha = chnl_dt/(chnl_dt + 1/(2*M_PI*chnl_switching_cutoff_freq));
  chnl_original = msg->channels[KNOB_CHANNEL-1];

  if(fabs(chnl_original) < 2500 && fabs(chnl_original) > 600)
  {
    chnl_switching += (chnl_original - chnl_switching)*alpha;
  }

  if(chnl_switching > CHANNEL_MID && SVO_INITIATED == false && SVO_RESET_SENT == false)
  {
    key_msg.data = "r";
    svo_remote_key_pub.publish(key_msg);
    sensor_fusion.resetVSLAM();
    SVO_RESET_SENT = true;
  }
  if(chnl_switching > CHANNEL_MID && SVO_INITIATED == false && SVO_RESET_SENT == true)
  {
    key_msg.data = "s";
    svo_remote_key_pub.publish(key_msg);

    SVO_INITIATED = true;
  }
  if(chnl_switching < CHANNEL_MID)
  {
    SVO_RESET_SENT = false;
    SVO_INITIATED = false;
  }
}

void visualizationMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  sensor_fusion.updateVSLAM_MapPointsRotationCorrection(msg);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  sensor_fusion.updateIMUData(msg);
}

void vslamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  sensor_fusion.updateVSLAMData(msg);

  Vector3f pos_ned;
  float psi_ned;

  // Converting position from NWU to NED
  pos_ned(0) = sensor_fusion.position.getX();
  pos_ned(1) = -sensor_fusion.position.getY();
  pos_ned(2) = -sensor_fusion.position.getZ();
  psi_ned = -sensor_fusion.psi;

  //check if lambda has been estimated and the camera rotation corrected
  if(sensor_fusion.isVSLAMEstimationHealthy())
  {
    publishSensorPos(msg->header.stamp, pos_ned(0), pos_ned(1), pos_ned(2), psi_ned, true);
  }
}

void px4flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
  float z = msg->ground_distance;
  sensor_fusion.updateSonarData(z, msg->header.stamp);

  Vector3f pos_ned;
  float psi_ned;
  // Converting position from NWU to NED
  pos_ned(0) = sensor_fusion.position.getX();
  pos_ned(1) = -sensor_fusion.position.getY();
  pos_ned(2) = -sensor_fusion.position.getZ();
  psi_ned = -sensor_fusion.psi;

  if(sensor_fusion.isVSLAMEstimationHealthy() == false)
  {
    publishSensorPos(msg->header.stamp, pos_ned(0), pos_ned(1), pos_ned(2), psi_ned, false);
    //For Debugging
//    publishSensPos(msg->header.stamp, 0, -3.4*sin(M_PI*tmp_count), ekf_z.x_hat_kplus1_kplus1(0,0), M_PI*5/6);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ekf_vslam_fusion");
  ros::NodeHandle n;

  diagnostic_updater::Updater health_monitor;

  health_monitor.setHardwareIDf("Nayan-%i Odroid", SYSTEM_ID);
  health_monitor.add("Sensor Health Monitor", &sensor_fusion, &SensorFusion::monitorSensorHealth);

  ros::Subscriber vslam_sub = n.subscribe("/svo/pose", 10, vslamCallback);
  ros::Subscriber stats_sub = n.subscribe("/mavros/rc/in", 10, channelCallback);
  ros::Subscriber rpy_sub = n.subscribe("/mavros/imu/data", 100, imuCallback);
  ros::Subscriber px4flow_sub = n.subscribe("/px4flow/opt_flow", 10, px4flowCallback);
  ros::Subscriber points_sub = n.subscribe("/svo/points", 100, visualizationMarkerCallback);

  sensor_fusion.pos_pub_vgnd = n.advertise<geometry_msgs::Vector3Stamped>("cv_pose", 100);
  sensor_fusion.acc_pub_hbf = n.advertise<geometry_msgs::Vector3Stamped>("sens_acc_hbf", 100);
  sensor_fusion.z_hat_pub = n.advertise<geometry_msgs::Vector3Stamped>("z_hat_ekf", 100);
  sensor_fusion.cv_pose_pub = n.advertise<geometry_msgs::PoseStamped>("sens_pose", 100);
  sensor_fusion.z_static_states_pub = n.advertise<geometry_msgs::Vector3Stamped>("z_static_states", 100);
  sensor_fusion.map_points_pub = n.advertise<visualization_msgs::Marker>("calib_points", 100);

  svo_remote_key_pub = n.advertise<std_msgs::String>("svo/remote_key", 5);

  int rate = 50;
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {

    //check for sensor callbacks
    ros::spinOnce();

    //mmonitor if all sensors are healthy
    health_monitor.update();

    //publish debug messages
    sensor_fusion.publishDebugMessages(ros::Time::now());

    //sleep for a particular amount of time
    loop_rate.sleep();
  }
}
