/**
 * @brief HILState plugin
 * @file control_state_estimate.cpp
 * @author Atulya Shivam Shree <atulyashivamshree@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace mavplugin {
	/**
	 * @brief HILState plugin
	 *
	 * This plugin can publish data from hil_state on FCU to ROS
	 */
	class HILStatePlugin : public MavRosPlugin {
	public:
		HILStatePlugin() :
			flow_nh("~hil_state"),
			uas(nullptr)
		{ };

		void initialize(UAS &uas_)
		{

			uas = &uas_;

			flow_nh.param<std::string>("frame_id", frame_id, "hil_state");

			pos_des_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("pos_desired", 10);
			vel_des_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("vel_desired", 10);
			pos_target_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("pos_target", 10);
			vel_target_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("vel_target", 10);
			accel_target_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("accel_target", 10);
		}

		const message_map get_rx_handlers() {
			return {
				       MESSAGE_HANDLER(MAVLINK_MSG_ID_HIL_STATE, &HILStatePlugin::handle_hil_state)
			};
		}

	private:
		ros::NodeHandle flow_nh;
		UAS *uas;

		std::string frame_id;

		ros::Publisher pos_des_pub;
		ros::Publisher vel_des_pub;
		ros::Publisher pos_target_pub;
		ros::Publisher vel_target_pub;
		ros::Publisher accel_target_pub;

		void handle_hil_state(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
			mavlink_hil_state_t state;
			mavlink_msg_hil_state_decode(msg, &state);

			//Acceleration
			auto pos_des_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			pos_des_msg->header.stamp = ros::Time::now();
			pos_des_msg->vector.x = state.roll;
			pos_des_msg->vector.y = state.pitch;
			pos_des_msg->vector.z = state.yaw;
			pos_des_pub.publish(pos_des_msg);

			// Velocity
			auto vel_des_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			vel_des_msg->header.stamp = ros::Time::now();
			vel_des_msg->vector.x = state.rollspeed;
			vel_des_msg->vector.y = state.pitchspeed;
			vel_des_msg->vector.z = state.yawspeed;
			vel_des_pub.publish(vel_des_msg);

			// Position
			auto pos_target_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			pos_target_msg->header.stamp = ros::Time::now();
			pos_target_msg->vector.x = state.lat;
			pos_target_msg->vector.y = state.lon;
			pos_target_msg->vector.z = state.alt;
			pos_target_pub.publish(pos_target_msg);

			auto vel_target_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			vel_target_msg->header.stamp = ros::Time::now();
			vel_target_msg->vector.x = state.vx;
			vel_target_msg->vector.y = state.vy;
			vel_target_msg->vector.z = state.vz;
			vel_target_pub.publish(vel_target_msg);

			auto accel_target_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			accel_target_msg->header.stamp = ros::Time::now();
			accel_target_msg->vector.x = state.xacc;
			accel_target_msg->vector.y = state.yacc;
			accel_target_msg->vector.z = state.zacc;
			accel_target_pub.publish(accel_target_msg);

		}
	};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::HILStatePlugin, mavplugin::MavRosPlugin)
