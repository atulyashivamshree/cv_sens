/*
 * marker_track.cpp
 *
 *  Created on: 22-Dec-2015
 *      Author: atulya
 */

#include "cv_sens/pattern_track.h"

#define ROC_XY                          1.1
#define ROC_Z                           0.4
#define ROC_YAW                         15*M_PI/180.0

geometry_msgs::PoseStamped pose_msg;
ros::Publisher waypoint_pub;

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

void publishWaypoint(Tracker track_marker)
{
  if(track_marker.isCalibrationFinished() == true)
  {
    float dist_xy = sqrt(pow(track_marker.marker_pos[0],2) + pow(track_marker.marker_pos[1],2));
    float dist_z = fabs((track_marker.marker_pos[2] + track_marker.target_height));

    if(dist_xy > ROC_XY || dist_z > ROC_Z)
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
        sendPositionWithYaw(track_marker.marker_pos[0], track_marker.marker_pos[1],
                            track_marker.marker_pos[2] + track_marker.target_height, 0);
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

void px4flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
  float z = msg->ground_distance;
//  rpy_imu_msg.vector.y = -z;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "marker_setpoint_gen");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  float target_height;
  pnh.param("z", target_height, float(2.0f));
  ROS_INFO("\nTarget height above marker is [%f]", target_height);

  Tracker marker_track(n, target_height);

  printf("\nKeep the quad over marker aligned with the front and press i\n");
  printf("To bypass calibration press b\n");

  ros::Subscriber vslam_sub = n.subscribe("marker_pose", 1, &Tracker::markerCallback, &marker_track);
//  ros::Subscriber stats_sub = n.subscribe("/mavros/rc/in", 10, statsCallback);
  ros::Subscriber imu_sub = n.subscribe("/mavros/imu/data", 1, &Tracker::imuCallback, &marker_track);
//  ros::Subscriber global_pos_sub = n.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
//  ros::Subscriber px4flow_sub = n.subscribe("/px4flow/opt_flow", 10, px4flowCallback);

  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 100);
  marker_track.rpy_imu_pub = n.advertise<geometry_msgs::Vector3Stamped>("rpy_imu", 100);
  marker_track.rpy_cam_pub = n.advertise<geometry_msgs::Vector3Stamped>("rpy_cam", 100);

  int rate = 10;
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    ros::spinOnce();

    marker_track.processUserAction();

    marker_track.publishDebugMessages();

    loop_rate.sleep();
  }
}
