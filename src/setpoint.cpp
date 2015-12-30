/*
 * setpoint.cpp
 *
 *  Created on: 09-Dec-2015
 *      Author: atulya
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include <boost/lexical_cast.hpp>
#include "sensor_fusion.h"

#define RELATIVE_MOVEMENT_XY    6       //maximum relative movement in the xy plane
#define RELATIVE_MOVEMENT_ALT   2       //maximum relative movement in the z plane

float chnl_switching = 0;
float chnl_original;
float chnl_switching_cutoff_freq = 0.6;
float chnl_switching_dt = 1/35.0f;

bool flag_waypoint_sent = false;

geometry_msgs::PoseStamped pose_msg;
ros::Publisher setpoint_pub;

void error_msg()
{
  printf("Incorrect usage. CORRECT Way : rosrun setpoint_enu -p 1.2 2.3 4.5\n -h for help");
}

void help_msg()
{
  printf("\nSend waypoints relative to current position. If current position is (2,3,4) sending (0,1,-1) will update the target waypoint on FCU to (2,3,3)\n");
}

void statsCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
  float alpha = chnl_switching_dt/(chnl_switching_dt + 1/(2*M_PI*chnl_switching_cutoff_freq));
  chnl_original = msg->channels[KNOB_CHANNEL-1];

  if(fabs(chnl_original) < 2500 && fabs(chnl_original) > 600)
  {
    chnl_switching += (chnl_original - chnl_switching)*alpha;
  }

  if(chnl_switching > CHANNEL_MID && flag_waypoint_sent == false)
  {
    flag_waypoint_sent = true;
    pose_msg.header.stamp = ros::Time::now();
    setpoint_pub.publish(pose_msg);
  }
  if(chnl_switching < CHANNEL_MID)
  {
    flag_waypoint_sent = false;
  }

}

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

  if(argc < 2)
  {
    error_msg();
    return -1;
  }
  std::string inp(argv[1]);

  ros::Rate loop_rate(10);
  ros::Subscriber stats_sub = n.subscribe("/mavros/rc/in", 10, statsCallback);
  setpoint_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 5);

  float px, py, pz;

  if(inp.compare("-p") == 0)
  {
    if(argc < 5)
    {
      error_msg();
      return -1;
    }
    try
    {
      px = boost::lexical_cast<float>(argv[2]);
      py = boost::lexical_cast<float>(argv[3]);
      pz = boost::lexical_cast<float>(argv[4]);
    }
    catch(std::exception &e)
    {
      printf("Error is %s",e.what());
      error_msg();
      return -1;
    }
  }
  else
  {
    if(inp.compare("-h") == 0)
    {
      help_msg();
      return -1;
    }
    else
    {
      error_msg();
      return -1;
    }
  }

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
