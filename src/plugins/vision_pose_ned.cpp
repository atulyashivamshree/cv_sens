/**
 * @brief Vision Pose Plugin
 * @file vision_pose_ned.cpp
 * @author Atulya Shivam Shree <atulyashivamshree@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 *
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace mavplugin {
/**
 * @brief Custom NED Vision Pose plugin
 */
class VisionPoseNEDPlugin : public MavRosPlugin {
public:
    VisionPoseNEDPlugin() :
        vision_pose_nh("~vision_pose_ned"),
        uas(nullptr)
    { };

    void initialize(UAS &uas_)
    {
        uas = &uas_;
        vision_pose_sub = vision_pose_nh.subscribe("pose", 10, &VisionPoseNEDPlugin::poseCallback, this);
    }

    const message_map get_rx_handlers()
    {
        return { /* Rx disabled */ };
    }

private:
    std::recursive_mutex mutex;
    ros::NodeHandle vision_pose_nh;
    UAS *uas;

    ros::Subscriber vision_pose_sub;

    /* -*- low-level send functions -*- */

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
        mavlink_message_t msg;

        tf::Quaternion q;
        tf::Matrix3x3 R;
        double roll, pitch, yaw;
        tf::quaternionMsgToTF(pose_msg->pose.orientation, q);
        R.setRotation(q);
        R.getRPY(roll, pitch, yaw);

        mavlink_msg_vision_position_estimate_pack_chan(UAS_PACK_CHAN(uas), &msg,
                 pose_msg->header.stamp.toNSec()/1000L,
                 pose_msg->pose.position.x,
                 pose_msg->pose.position.y,
                 pose_msg->pose.position.z,
                 roll,
                 pitch,
                 yaw);
        UAS_FCU(uas)->send_message(&msg);
    }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VisionPoseNEDPlugin, mavplugin::MavRosPlugin)

