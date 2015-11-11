/**
 * @brief Attitude plugin
 * @file control_state_estimate.cpp
 * @author Atulya Shivam Shree <atulyashivamshree@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Vector3Stamped.h>

namespace mavplugin {
	/**
	 * @brief Attitude plugin
	 *
	 * This plugin can publish data from attitude on FCU to ROS
	 */
	class AttitudePlugin : public MavRosPlugin {
	public:
		AttitudePlugin() :
			flow_nh("~attitude_ned"),
			uas(nullptr)
		{ };

		void initialize(UAS &uas_)
		{
			uas = &uas_;

			flow_nh.param<std::string>("frame_id", frame_id, "attitude_ned");

			attitude_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("attitude", 10);
			angular_velocity_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("angular_velocity", 10);
		}

		const message_map get_rx_handlers() {
			return {
				       MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE, &AttitudePlugin::handle_attitude)
			};
		}

	private:
		ros::NodeHandle flow_nh;
		UAS *uas;

		std::string frame_id;

		ros::Publisher attitude_pub;
		ros::Publisher angular_velocity_pub;

		void handle_attitude(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
			mavlink_attitude_t state;
			mavlink_msg_attitude_decode(msg, &state);

			auto attitude_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			attitude_msg->header.stamp = ros::Time::now();
			attitude_msg->vector.x = state.roll*180/3.14;
			attitude_msg->vector.y = state.pitch*180/3.14;
			attitude_msg->vector.z = state.yaw*180/3.14;
			attitude_pub.publish(attitude_msg);

			auto angular_velocity_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			angular_velocity_msg->header.stamp = ros::Time::now();
			angular_velocity_msg->vector.x = state.rollspeed;
			angular_velocity_msg->vector.y = state.pitchspeed;
			angular_velocity_msg->vector.z = state.yawspeed;
			angular_velocity_pub.publish(angular_velocity_msg);
		}
	};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::AttitudePlugin, mavplugin::MavRosPlugin)
