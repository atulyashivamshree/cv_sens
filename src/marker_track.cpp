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

geometry_msgs::Vector3Stamped waypoint_msg;
ros::Publisher waypoint_pub;

void markerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	waypoint_msg.vector.x = msg->pose.position.z;
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	waypoint_msg.vector.y = msg->altitude;
}

void statsCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
}

void px4flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
  float z = msg->ground_distance;
  waypoint_msg.vector.z = z;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "marker_setpoint_gen");
  ros::NodeHandle n;

  ros::Subscriber vslam_sub = n.subscribe("marker_pose", 10, markerCallback);
  ros::Subscriber stats_sub = n.subscribe("/mavros/rc/in", 10, statsCallback);
  ros::Subscriber global_pos = n.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
  ros::Subscriber px4flow_sub = n.subscribe("/px4flow/opt_flow", 10, px4flowCallback);

  waypoint_pub = n.advertise<geometry_msgs::Vector3Stamped>("waypoint", 100);

  int rate = 25;
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {

    ros::spinOnce();

    waypoint_msg.header.stamp = ros::Time::now();
    waypoint_pub.publish(waypoint_msg);

    loop_rate.sleep();
  }
}
