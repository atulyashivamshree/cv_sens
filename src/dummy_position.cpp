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
 * @param psi yaw angle in radians
 * @param flag_cv_active if true indicates that XYZ estimates are active, if false
 *             indicates that computer vision estimates for XY are inactive and only
 *             Z coordinates from sonar are good
 */
void publishSensorPos(ros::Time stamp, float x, float y, float z, float psi, bool flag_cv_active)
{
  tf::Matrix3x3 R;
  /*FIXME BUG in UAS
   * Transformation in Eigen/Geometry near mat.eulerAngles it has been stated
   * the returned angles are in the range [0:pi]x[-pi:pi]x[-pi:pi]
   * In quaternion_to_rpy() present in the file uas_quaternion_utils.cpp in the folder
   * mavros/mavros/src/lib the function mat.eulerAngles has been used to obtain YPR in
   * the order 2,1,0
   * So the yaw obtained is in the range is in [0 pi] which causes a bug
   * Hence the yaw obtained inside the FCU code is in range of [0 pi] which is wrong coz
   * -pi/2 was represented as pi/2
   * So now the yaw angle is sent as roll angle and similarly in FCU the cv yaw is assigned
   * to the roll angle
   * WARNING: Do not send anything through this message without cross-checking
   */

  if(flag_cv_active)
    R.setRPY(psi,0.7,-1);                 //Hard coded similar thing on HLP implyinng CV is active
  else
    R.setRPY(psi,0.7,-1.35);              //Hard coded similar thing on HLP implying CV in inactive

  tf::Quaternion q;
  R.getRotation(q);
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

  float px = 1, py = 2 , pz = 3, psi = 1.57;

  ros::Rate loop_rate(20);
  position_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 200);

//  for(int i = 0; i<5 ; i++)
  while(ros::ok())
  {
    ros::spinOnce();

    ros::Time stamp_now = ros::Time::now();
    publishSensorPos(stamp_now, px, py, pz, psi, true);
    position_pub.publish(pose_msg);
    ROS_INFO("Sending Position and Yaw : [%.3f, %.3f, %.3f]; [%.3f]", px, py, pz, psi);

    loop_rate.sleep();
  }

}
