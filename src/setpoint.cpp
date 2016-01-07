/*
 * setpoint.cpp
 *
 *  Created on: 09-Dec-2015
 *      Author: atulya
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <mavros_msgs/RCIn.h>

#define RELATIVE_MOVEMENT_XY    6       //maximum relative movement in the xy plane
#define RELATIVE_MOVEMENT_ALT   2       //maximum relative movement in the z plane

bool flag_waypoint_sent = false;

geometry_msgs::PoseStamped pose_msg;
ros::Publisher setpoint_pub;

//restrict relative movements to within certain range
void restrictRelativePosition(float &x, float &y, float &z)
{
  float rel_dist = sqrt(x*x + y*y);
  if(rel_dist > RELATIVE_MOVEMENT_XY && rel_dist > 0)
  {
    x = x*RELATIVE_MOVEMENT_XY/rel_dist;
    y = y*RELATIVE_MOVEMENT_XY/rel_dist;
  }

  float rel_height = fabs(z);
  if(rel_height > RELATIVE_MOVEMENT_ALT && rel_height > 0)
    z = z*RELATIVE_MOVEMENT_ALT/rel_height;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "setpoint_enu");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  float px, py, pz;

  ros::Rate loop_rate(10);
  setpoint_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 5);

  pnh.param("x", px, float(0.0f));
  pnh.param("y", py, float(0.0f));
  pnh.param("z", pz, float(0.0f));

  restrictRelativePosition(px, py, pz);

  pose_msg.pose.orientation.w = 1;
  pose_msg.pose.orientation.x = 0;
  pose_msg.pose.orientation.y = 0;
  pose_msg.pose.orientation.z = 0;
  pose_msg.pose.position.x = px;
  pose_msg.pose.position.y = py;
  pose_msg.pose.position.z = pz;

  for(int i = 0; i<5 ; i++)
//  while(ros::ok())
  {
    ros::spinOnce();

    pose_msg.header.stamp = ros::Time::now();
    setpoint_pub.publish(pose_msg);
    ROS_INFO("Sending Waypoint : [%.3f, %.3f, %.3f]", px, py, pz);

    loop_rate.sleep();
  }

}
