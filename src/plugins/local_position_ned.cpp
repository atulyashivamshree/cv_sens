/**
 * @brief LocalPositionNED plugin
 * @file local_position_ned.cpp
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
	 * @brief LocalPositionNED plugin
	 *
	 * This plugin can publish data from local_position_ned on FCU to ROS
	 */
	class LocalPositionNEDPlugin : public MavRosPlugin {
	public:
		LocalPositionNEDPlugin() :
			flow_nh("~local_position_ned"),
			uas(nullptr)
		{ };

		void initialize(UAS &uas_)
		{
			uas = &uas_;

			flow_nh.param<std::string>("frame_id", frame_id, "local_position_ned");

			position_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("position", 10);
			velocity_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("velocity", 10);
		}

		const message_map get_rx_handlers() {
			return {
				       MESSAGE_HANDLER(MAVLINK_MSG_ID_LOCAL_POSITION_NED, &LocalPositionNEDPlugin::handle_local_position_ned)
			};
		}

	private:
		ros::NodeHandle flow_nh;
		UAS *uas;

		std::string frame_id;

		ros::Publisher position_pub;
		ros::Publisher velocity_pub;

		void handle_local_position_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
			mavlink_local_position_ned_t state;
			mavlink_msg_local_position_ned_decode(msg, &state);

			auto position_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			position_msg->header.stamp = ros::Time::now();
			position_msg->vector.x = state.x;
			position_msg->vector.y = state.y;
			position_msg->vector.z = state.z;
			position_pub.publish(position_msg);

			auto velocity_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			velocity_msg->header.stamp = ros::Time::now();
			velocity_msg->vector.x = state.vx;
			velocity_msg->vector.y = state.vy;
			velocity_msg->vector.z = state.vz;
			velocity_pub.publish(velocity_msg);
		}
	};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LocalPositionNEDPlugin, mavplugin::MavRosPlugin)
