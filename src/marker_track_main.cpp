/*
 * marker_track.cpp
 *
 *  Created on: 22-Dec-2015
 *      Author: atulya
 */

#include "cv_sens/pattern_track.h"

#include <sensor_msgs/NavSatFix.h>
#include <px_comm/OpticalFlow.h>
#include <mavros_msgs/RCIn.h>

#include <ros/param.h>

//TODO remove unnecessary topics

#define ROC_XY                          0.7
#define ROC_Z                           0.6
#define ROC_YAW                         15*M_PI/180.0
#define WP_TIME_DIFF                    4

geometry_msgs::PoseStamped g_pose_msg;
ros::Publisher g_waypoint_pub;

geometry_msgs::Vector3Stamped g_relative_target_msg;
ros::Publisher g_relative_target_pub;

//stores the desired position and time duration of stay at that point
typedef std::pair<tf::Vector3, float> trajectory_point;

class Trajectory
{
  std::vector<trajectory_point> landing_seq_;
  trajectory_point curr_point_;
  int iter_;
  ros::Time last_wp_sending_time_;
  float waypoint_hovering_time_;

private:
  /**
   * @brief returns true if the waypoint has been covered
   * @param pt desired waypoint in the marker frame
   * @param curr current position in the marker frame
   * @param dt time difference between function calls
   */
  bool hasReachedPoint(const trajectory_point &pt, const tf::Vector3 &curr, float dt) {

    tf::Vector3 diff = pt.first - curr;
    float dist_xy = sqrt(pow(diff.getX(),2) + pow(diff.getY(),2));
    float dist_z = fabs(diff.getZ());

    if(dist_xy < ROC_XY && dist_z < ROC_Z)
      waypoint_hovering_time_ += dt;
    else
    {
      if(waypoint_hovering_time_ >= 0.0f)
        waypoint_hovering_time_ -= dt;
    }

    if(waypoint_hovering_time_ >= pt.second)
    {
      waypoint_hovering_time_ = 0;
      return true;
    }
    else
      return false;
  }

public:

  Trajectory()
  {
    iter_ = 0;
    last_wp_sending_time_ = ros::Time::now();
    waypoint_hovering_time_ = 0;
  }

  /**
   * @brief inserts a point into the landing trajectory
   */
  void insertPoint(trajectory_point point) {
    landing_seq_.push_back(point);
  }

  void getSequence(ros::NodeHandle pnh);

  /**
   * @brief updates the landing sequence based on the pattern tracking algorithm
   */
  void updateTrajectory(const Pattern_Tracker &track_marker, float dt);
};

void sendPositionWithYaw(float x, float y, float z, float yaw)
{
  tf::Matrix3x3 R;
  R.setRPY(0,0,yaw);
  tf::Quaternion q;
  R.getRotation(q);
  tf::quaternionTFToMsg(q, g_pose_msg.pose.orientation);

  //Sending position output to FCU
  g_pose_msg.header.stamp = ros::Time::now();
  g_pose_msg.pose.position.x = x;
  g_pose_msg.pose.position.y = y;
  g_pose_msg.pose.position.z = z;
  g_waypoint_pub.publish(g_pose_msg);

}

void Trajectory::updateTrajectory(const Pattern_Tracker &track_marker, float dt)
{
  if(track_marker.isCalibrationFinished() == true)
  {
    tf::Vector3 curr_pos = track_marker.getCameraPosInMarkerFrame();

    tf::vector3TFToMsg(landing_seq_[iter_].first,g_relative_target_msg.vector);

    if(hasReachedPoint(landing_seq_[iter_],curr_pos, dt ))
    {
      ROS_INFO("Landing sequence waypoint no [%d] with coordinates [%.3f, %.3f, %.3f] reached", iter_,
               curr_pos.getX(), curr_pos.getY(), curr_pos.getZ());
      if(iter_ < landing_seq_.size() - 1)
        iter_++;
      else
        ROS_INFO("Landing sequence complete");
    }
    else
    {
      ros::Duration wp_time_diff = ros::Time::now() - last_wp_sending_time_;
      if(wp_time_diff.toSec() > WP_TIME_DIFF)
      {
        last_wp_sending_time_ = ros::Time::now();
        tf::Vector3 des_wp = track_marker.getDesiredMovementToReachTarget(landing_seq_[iter_].first);
        ROS_INFO("Sending out waypoint [%.3f, %.3f, %.3f] ", des_wp.getX(), des_wp.getY(), des_wp.getZ());
        sendPositionWithYaw(des_wp.getX(), des_wp.getY(), des_wp.getZ(), 0);
      }
    }
  }
}
/**
 * @brief attempts to read trajectory points from a rosparam yaml file. If failed it assigns default values
 */
void Trajectory::getSequence(ros::NodeHandle pnh)
{
  XmlRpc::XmlRpcValue waypoint_list;

  if(pnh.getParam("waypoint", waypoint_list))
  {
//    ROS_INFO("tYPE IS %d", waypoint_list.getType());
    if(waypoint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {

  //    ROS_INFO("GOT WP ");
  //    ROS_INFO("tYPE IS %d and size is %d", waypoint_list[0].getType(), waypoint_list.size());

      for(int i = 0; i < waypoint_list.size(); i++)
      {
        if(waypoint_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {

          XmlRpc::XmlRpcValue::iterator veci;

          if(waypoint_list[i].size() >= 1)
          {
            for(veci = waypoint_list[i].begin(); veci!=waypoint_list[i].end(); veci++)
            {
              XmlRpc::XmlRpcValue pos;
              pos = veci->second;
//              ROS_INFO("tYPE IS %d size is %d", pos.getType(), pos.size());

              trajectory_point pt;

              if(pos.getType() != XmlRpc::XmlRpcValue::TypeArray)
                continue;

              if(pos.size()!=4)
                continue;

              if(pos[0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
                 pos[1].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
                 pos[2].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
                 pos[3].getType() != XmlRpc::XmlRpcValue::TypeDouble)
                continue;

              pt.first = tf::Vector3(pos[0], pos[1], pos[2]);
              pt.second = (double)pos[3];

              landing_seq_.push_back(pt);
              ROS_INFO("WP %d : [%.3f, %.3f, %.3f] Time : [%.3f]",
                          (int)landing_seq_.size(),
                          pt.first.getX(), pt.first.getY(), pt.first.getZ(),
                          pt.second);
            }
          }
        }
      }
    }
    return;
  }

  ROS_WARN("Got no trajectory points as parameter. Using default value of [0, 0, 2]");
  trajectory_point pt;
  pt.first = tf::Vector3(0.0f, 0.0f, 2.0f);
  pt.second = 5.0f;
  landing_seq_.push_back(pt);
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

  float target_height, marker_size;
  int marker_id;
  pnh.param("z", target_height, float(2.0f));
  pnh.param("marker_size", marker_size, float(0.284));
  pnh.param("marker_id", marker_id, int(26));
  ROS_INFO("Tracking Aruco marker with id %d and size %.3f", marker_id, marker_size);

  Pattern_Tracker marker_track(n, target_height, marker_size, marker_id);
  Trajectory marker_trajectory;
  marker_trajectory.getSequence(pnh);

  printf("\nKeep the quad over marker aligned with the front and press i\n");
  printf("To bypass calibration press b\n");

  ros::Subscriber vslam_sub = n.subscribe("marker_pose", 1, &Pattern_Tracker::markerCallback, &marker_track);

//  ros::Subscriber stats_sub = n.subscribe("/mavros/rc/in", 10, statsCallback);
//  ros::Subscriber global_pos_sub = n.subscribe("/mavros/global_position/global", 10, globalPositionCallback);
//  ros::Subscriber px4flow_sub = n.subscribe("/px4flow/opt_flow", 10, px4flowCallback);

  g_waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 100);
  g_relative_target_pub = n.advertise<geometry_msgs::Vector3Stamped>("relative_target", 100);

  int rate = 2;
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    //check for any callbacks
    ros::spinOnce();

    //check for any user based keyboard inputs
    marker_track.processUserAction();

    //update the landing sequence depending on availibility of marker
    if(marker_track.flag_new_marker_received)
    {
      marker_trajectory.updateTrajectory(marker_track, 1.0f/rate);
      ROS_DEBUG("Updating the landing sequence");
      marker_track.flag_new_marker_received = false;
    }

    g_relative_target_msg.header.stamp = ros::Time::now();
    g_relative_target_pub.publish(g_relative_target_msg);
    marker_track.publishDebugMessages();

    loop_rate.sleep();
  }
}
