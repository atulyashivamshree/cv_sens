/*
 * setpoint.cpp
 *
 *  Created on: 09-Dec-2015
 *      Author: atulya
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

geometry_msgs::PoseStamped pose_msg;
ros::Publisher position_pub;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "setpoint_enu");
  ros::NodeHandle n;

  float px = 1, py = 2 , pz = 3;

  ros::Rate loop_rate(20);
  position_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 200);

  pose_msg.pose.orientation.w = 1;
  pose_msg.pose.orientation.x = 0;
  pose_msg.pose.orientation.y = 0;
  pose_msg.pose.orientation.z = 0;
  pose_msg.pose.position.x = px;
  pose_msg.pose.position.y = py;
  pose_msg.pose.position.z = pz;

//  for(int i = 0; i<5 ; i++)
  while(ros::ok())
  {
    ros::spinOnce();

    pose_msg.header.stamp = ros::Time::now();
    position_pub.publish(pose_msg);
    ROS_INFO("Sending Position : [%.3f, %.3f, %.3f]", px, py, pz);

    loop_rate.sleep();
  }

}
