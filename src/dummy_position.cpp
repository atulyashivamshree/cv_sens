/*
 * setpoint.cpp
 *
 *  Created on: 09-Dec-2015
 *      Author: atulya
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

geometry_msgs::PoseStamped pose_msg;
ros::Publisher position_pub;

//This function is the final link that sends current position estimates and yaw estimate to the flight control unit
/**
 * @brief sends the current position and yaw estimate to the flight controller
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
  tf::quaternionTFToMsg(q, pose_msg.pose.orientation);
  pose_msg.header.stamp = stamp;
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;

  position_pub.publish(pose_msg);
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "setpoint_enu");
  ros::NodeHandle n;
  position_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose_ned/pose", 200);

  ros::NodeHandle pnh("~");
  float px = 1, py = 2 , pz = 3;
  float roll, pitch, yaw;
  int rate;

  pnh.param("x", px, float(1.0f));
  pnh.param("y", py, float(2.0f));
  pnh.param("z", pz, float(3.0f));
  pnh.param("yaw", yaw, float(1.5f));
  pnh.param("rate", rate, int(20));

  ros::Rate loop_rate(rate);

  while(ros::ok())
  {

    ros::Time stamp_now = ros::Time::now();
    publishSensorPos(stamp_now, px, py, pz, yaw, true);

    ROS_INFO("Pos [%.3f, %.3f, %.3f]; Y: [%.3f]", px, py, pz, yaw);
    loop_rate.sleep();
  }

}
