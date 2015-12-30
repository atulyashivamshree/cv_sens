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

#include <mavros_msgs/RCIn.h>

#define ROC_XY          1.1
#define ROC_Z           0.4
#define ROC_YAW         15*M_PI/180.0

geometry_msgs::PoseStamped pose_msg;
ros::Publisher waypoint_pub;

float target_pos[3];
float marker_pos[3];

void error_msg()
{
  printf("Incorrect usage. CORRECT Way : rosrun trak_marker 1.2 2.3 4.5");
}

void sendPositionWithYaw(float x, float y, float z, float yaw)
{
  tf::Matrix3x3 R;
  R.setRPY(0,0,yaw);
  tf::Quaternion q;
  R.getRotation(q);
  tf::quaternionTFToMsg(q, pose_msg.pose.orientation);

  //Sending position output to FCU
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;
  waypoint_pub.publish(pose_msg);

}

void markerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double phi_v, theta_v, psi_v;
  tf::Matrix3x3 R_b_to_VGND;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.orientation, q);
  R_b_to_VGND.setRotation(q);
  R_b_to_VGND.getRPY(phi_v, theta_v, psi_v);

  ROS_INFO("RPY is [%f, %f, %f]", phi_v, theta_v, psi_v);

  marker_pos[0] = msg->pose.position.x;
  marker_pos[1] = msg->pose.position.y;
  marker_pos[2] = msg->pose.position.z;

  float dist_xy = sqrt(pow(marker_pos[0] - target_pos[0],2) + pow(marker_pos[1] - target_pos[1],2));
  float dist_z = fabs((marker_pos[2] - target_pos[2]));

  if(dist_xy > ROC_XY || dist_z > ROC_Z || 1)
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

void px4flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
  float z = msg->ground_distance;
//  waypoint_msg.vector.z = z;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "marker_setpoint_gen");
  ros::NodeHandle n;

  if(argc < 4)
  {
    error_msg();
    return -1;
  }
  try
  {
    target_pos[0] = boost::lexical_cast<float>(argv[1]);
    target_pos[1] = boost::lexical_cast<float>(argv[2]);
    target_pos[2] = boost::lexical_cast<float>(argv[3]);
  }
  catch(std::exception &e)
  {
    printf("Error is %s",e.what());
    error_msg();
    return -1;
  }

  ROS_INFO("Target position above waypoint is [%f, %f, %f]", target_pos[0], target_pos[1], target_pos[2]);

  ros::Subscriber vslam_sub = n.subscribe("marker_pose", 10, markerCallback);
  ros::Subscriber stats_sub = n.subscribe("/mavros/rc/in", 10, statsCallback);
  ros::Subscriber global_pos = n.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
  ros::Subscriber px4flow_sub = n.subscribe("/px4flow/opt_flow", 10, px4flowCallback);

  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 100);

  int rate = 5;
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    ros::spinOnce();

    sendPositionWithYaw(1,2,3,-3);

    loop_rate.sleep();
  }
}
