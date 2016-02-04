/*
 * marker_track.cpp
 *
 *  Created on: 22-Dec-2015
 *      Author: atulya
 */

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <px_comm/OpticalFlow.h>

#include <vikit/user_input_thread.h>

#include <mavros_msgs/RCIn.h>

#define ROC_XY                          2.0f
#define ROC_Z                           0.4
#define ROC_YAW                         15*M_PI/180.0
#define WP_SEND_INTERVAL                5.0f

#define CALIBRATION_INLIER_ERR_THRSH    9.0*M_PI/180.0f
#define CALIBRATION_NUM_VARIABLES       20                      //assuming 10 sec @2Hz
#define CALIBRATION_INLIER_PERCENT      75

boost::shared_ptr<vk::UserInputThread> keyboard_input;

geometry_msgs::PoseStamped pose_msg;
ros::Publisher waypoint_pub;

geometry_msgs::Vector3Stamped rpy_imu_msg;
ros::Publisher rpy_imu_pub;

geometry_msgs::Vector3Stamped rpy_cam_msg;
ros::Publisher rpy_cam_pub;

ros::Time prev_wp_time;

//All possible rotation matrices according to the different frames
tf::Matrix3x3 R_CB_EM, R_EB_CIB, R_EE_EB, R_EE_EM, R_CB_CIB, R_EE_EHBF;
tf::Matrix3x3 R_CB_CIB_ideal;

bool flag_calibration_started = false;
bool flag_calibration_image_acquired = false;
bool flag_calibration_finished = false;
int calibration_count = 0;
int calibration_inlier_count = 0;

float target_height;
float marker_pos[3];

void initializeRotation()
{
  R_EE_EM = R_EE_EB;
  flag_calibration_started = true;
  double phi, theta, psi;
  R_EE_EM.getRPY(phi, theta, psi);
  ROS_INFO("Calibration Initiated\n Rot from IMU is [R: %.3f, P: %.3f, Y: %.3f]\n Press C when there is good tracking to calibrate rotation correction\n",
                                     phi, theta, psi);
}

void printRotationMatrix(tf::Matrix3x3 R)
{
  printf("[%f. %f. %f\n%f, %f, %f\n %f, %f, %f]\n",
               R.getRow(0).getX(), R.getRow(0).getY(), R.getRow(0).getZ(),
               R.getRow(1).getX(), R.getRow(1).getY(), R.getRow(1).getZ(),
               R.getRow(2).getX(), R.getRow(2).getY(), R.getRow(2).getZ());
}

void useImageForCalibration()
{

  double phi_v, theta_v, psi_v;

  flag_calibration_image_acquired = true;

  R_CB_CIB = R_CB_EM*(R_EE_EM.transpose())*R_EE_EB*R_EB_CIB;
//  ROS_INFO("Rotation matrix R_CB_EM is");
//  printRotationMatrix(R_CB_EM);
//  ROS_INFO("Rotation matrix R_EM_EE is");
//  printRotationMatrix(R_EE_EM.transpose());
//  ROS_INFO("Rotation matrix R_EB_EE is");
//  printRotationMatrix(R_EE_EB.transpose());
//  ROS_INFO("Rotation matrix R_CIB_EB is");
//  printRotationMatrix(R_EB_CIB.transpose());

  R_CB_CIB.transpose().getRPY(phi_v, theta_v, psi_v);

  ROS_INFO("Rotation from camera ideal to camera is [R: %.3f, P: %.3f, Y: %.3f]",
                                       phi_v, theta_v, psi_v);
}

void bypassCalibration()
{
  flag_calibration_finished = true;
  ROS_INFO("Bypassing calibration: Assuming that camera is vertical");
}

void sendPositionWithYaw(float x, float y, float z, float yaw)
{
  tf::Matrix3x3 R;
  R.setRPY(0,0,yaw);
  tf::Quaternion q;
  R.getRotation(q);
  tf::quaternionTFToMsg(q, pose_msg.pose.orientation);

  //Sending position output to FCU
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;
  waypoint_pub.publish(pose_msg);

}

void markerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double phi_v, theta_v, psi_v;
  double phi, theta, psi;

  tf::Matrix3x3 R_aruco_correction;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.orientation, q);
  R_CB_EM.setRotation(q);
  R_aruco_correction.setRPY(M_PI/2, 0, M_PI);
  R_CB_EM = R_CB_EM*R_aruco_correction;             // In aruco_ros package; file aruco_ros_utils.cpp has a function
                                                    //arucoMarker2Tf which multiplies the rotation matrix by this value
                                                    // before publishing it. Hence the correction is applied here

//  ROS_INFO("Rotation matrix R_CB_EM is");
//  printRotationMatrix(R_CB_EM);
  tf::Matrix3x3 R_eb_em_from_cam = R_EB_CIB*R_CB_CIB.transpose()*R_CB_EM;
  R_eb_em_from_cam.transpose().getRPY(phi_v, theta_v, psi_v);   //this actually gives EB to EE RPY
//  ROS_INFO("Rotation from camera is [Roll: %f, Pitch: %f, Yaw: %f]",
//                                       phi_v, theta_v, psi_v);
//  rpy_cam_msg.vector.x = phi_v;
//  rpy_cam_msg.vector.y = theta_v;
//  rpy_cam_msg.vector.z = psi_v;

//  ROS_INFO("Rotation matrix R_EB_EE is");
//  printRotationMatrix(R_EE_EB.transpose());
  tf::Matrix3x3 R_eb_em_from_imu = R_EE_EB.transpose()*R_EE_EM;
  R_eb_em_from_imu.transpose().getRPY(phi, theta, psi);   //this actually gives EB to EE RPY
//  ROS_INFO("Rotation from IMU is [Roll: %f, Pitch: %f, Yaw: %f]",
//                                     phi, theta, psi);

//  rpy_imu_msg.vector.x = phi;
//  rpy_imu_msg.vector.y = theta;
//  rpy_imu_msg.vector.z = psi;

  if(fabs(fabs(psi - psi_v) - (2*M_PI)) < 0.2 )
  {
    if(psi<0)
      psi += 2*M_PI;
    else
      psi_v += 2*M_PI;
  }

  float meas_diff = sqrt(pow((phi_v - phi),2) + pow((theta_v - theta),2) + pow((psi_v - psi),2));
//  rpy_imu_msg.vector.y = meas_diff;

//  ROS_INFO("angle diff IMU and camera : %f, %d, %d", meas_diff, calibration_count, calibration_inlier_count);

  if(flag_calibration_finished == false && flag_calibration_started == true && flag_calibration_image_acquired == true)
  {
    if(calibration_count > CALIBRATION_NUM_VARIABLES)
    {
      ROS_INFO("CALIBRATION UNSUCCESSFUL : Press C to acquire another image and wait till a success is shown");
      flag_calibration_image_acquired = false;
    }
    calibration_count++;
    if(meas_diff < CALIBRATION_INLIER_ERR_THRSH)
      calibration_inlier_count++;

    if(calibration_inlier_count >= CALIBRATION_INLIER_PERCENT/100.0f*CALIBRATION_NUM_VARIABLES)
    {
      ROS_INFO("CALIBRATION SUCCESSFUL :\nNow the marker can be freely moved anywhere  (%d of %d are inliers)",
               calibration_inlier_count, calibration_count);
      flag_calibration_finished = true;
    }
  }

  tf::Vector3 r_cb, r_ee, r_ee_ideal, r_ehbf, r_ehbf_ideal, r_em;                                             //position of marker center with respect to the EE frame attached to quad
  tf::pointMsgToTF(msg->pose.position, r_cb);
  r_ee = R_EE_EB*R_EB_CIB*R_CB_CIB.transpose()*r_cb;
  r_ee_ideal = R_EE_EB*R_EB_CIB*R_CB_CIB_ideal.transpose()*r_cb;
//  r_em = R_EE_EM.transpose()*R_EE_EB*R_EB_CIB*R_CB_CIB.transpose()*(-1*r_cb);                   //camera with respect to the marker

  r_ehbf = R_EE_EHBF.transpose()*r_ee;
  r_ehbf_ideal = R_EE_EHBF.transpose()*r_ee_ideal;
  rpy_imu_msg.vector.x = r_cb.getX();
  rpy_imu_msg.vector.y = r_cb.getY();
  rpy_imu_msg.vector.z = r_cb.getZ();
////  ROS_INFO("desired position is [%f, %f, %f]", r_ehbf.getX(), r_ehbf.getY(), r_ehbf.getZ());
//  rpy_imu_msg.vector.x = r_ehbf_ideal.getX();
//  rpy_imu_msg.vector.y = r_ehbf_ideal.getY();
//  rpy_imu_msg.vector.z = r_ehbf_ideal.getZ();

  rpy_cam_msg.vector.x = r_ehbf.getX();
  rpy_cam_msg.vector.y = r_ehbf.getY();
  rpy_cam_msg.vector.z = r_ehbf.getZ();

//  ROS_INFO("RPY is [%f, %f, %f]", phi_v, theta_v, psi_v);
  marker_pos[0] = r_ehbf.getX();
  marker_pos[1] = r_ehbf.getY();
  marker_pos[2] = r_ehbf.getZ();

  if(flag_calibration_finished == true)
  {
    float dist_xy = sqrt(pow(marker_pos[0],2) + pow(marker_pos[1],2));
    float dist_z = fabs((marker_pos[2] + target_height));

    ros::Time t_now = ros::Time::now();
    ros::Duration delt = t_now - prev_wp_time;

    if(dist_xy > ROC_XY || dist_z > ROC_Z )
    {
      //Sending yaw output to FCU
      float dist_yaw;

      //check deviation of yaw from desired value
      if(dist_yaw > ROC_YAW)
      {
        sendPositionWithYaw(0,0,0,0);
      }
      else
      {
        if(delt.toSec() > WP_SEND_INTERVAL)
        {
          prev_wp_time = t_now;
          sendPositionWithYaw(marker_pos[0], marker_pos[1],
                            marker_pos[2] + target_height, 0);
        }
      }
    }
  }
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
//  waypoint_msg.vector.y = msg->altitude;
}

void statsCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  double phi, theta, psi;

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q);

  R_EE_EB.setRotation(q);
  R_EE_EB.getRPY(phi, theta, psi);
//  ROS_INFO("Rotation from IMU is [R: %.3f, P: %.3f, Y: %.3f]",
//                                         phi, theta, psi);
  R_EE_EHBF.setRPY(0.0f ,0.0f , psi);
}

void px4flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
  float z = msg->ground_distance;
//  rpy_imu_msg.vector.y = -z;
}

void processUserAction()
{
  if(keyboard_input!= NULL)
  {
    char terminal_input = keyboard_input->getInput();
    if(terminal_input != 0)
    {
      switch(terminal_input)
      {
        case 'i':
          initializeRotation();
          break;
        case 'c':
          useImageForCalibration();
          break;
        case 'b':
          bypassCalibration();
        default:
          break;
      }
    }
  }
}

void publishDebugMessages()
{
  rpy_cam_pub.publish(rpy_cam_msg);
  rpy_imu_msg.header.stamp = ros::Time::now();
  rpy_cam_msg.header.stamp = ros::Time::now();
  rpy_imu_pub.publish(rpy_imu_msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "marker_setpoint_gen");
  ros::NodeHandle n;

  keyboard_input = boost::make_shared<vk::UserInputThread>();

  ros::NodeHandle pnh("~");
  pnh.param("z", target_height, float(2.7f));

  R_EB_CIB.setRPY(M_PI , 0.0f, -M_PI/2);
  R_CB_CIB_ideal.setRPY(0.0f, 0.0f, 0.0f);
  R_CB_CIB.setRPY(0.0f,0.0f, 0.0f);
  R_CB_EM.setRPY(0.0f,0.0f, 0.0f);
  R_EE_EB.setRPY(0.0f,0.0f, 0.0f);
  R_EE_EM.setRPY(0.0f,0.0f, 0.0f);

  ROS_INFO("\nTarget height above marker is [%f]", target_height);
  printf("\nKeep the quad over marker aligned with the front and press i\n");
  printf("To bypass calibration press b\n");

  prev_wp_time = ros::Time::now();

  ros::Subscriber vslam_sub = n.subscribe("marker_pose", 1, markerCallback);
//  ros::Subscriber stats_sub = n.subscribe("/mavros/rc/in", 10, statsCallback);
  ros::Subscriber imu_sub = n.subscribe("/mavros/imu/data", 1, imuCallback);
//  ros::Subscriber global_pos_sub = n.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
//  ros::Subscriber px4flow_sub = n.subscribe("/px4flow/opt_flow", 10, px4flowCallback);

  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 10);
  rpy_imu_pub = n.advertise<geometry_msgs::Vector3Stamped>("rpy_imu", 10);
  rpy_cam_pub = n.advertise<geometry_msgs::Vector3Stamped>("rpy_cam", 10);

  int rate = 10;
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    ros::spinOnce();

    processUserAction();

    publishDebugMessages();

    loop_rate.sleep();
  }
  keyboard_input->stop();
}
