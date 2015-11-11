/**
 * @brief SimState plugin
 * @file control_state_estimate.cpp
 * @author Atulya Shivam Shree <atulyashivamshree@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/OpticalFlowRad.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace mavplugin {
	/**
	 * @brief SimState plugin
	 *
	 * This plugin can publish data from sim_state on FCU to ROS
	 */
	class SimStatePlugin : public MavRosPlugin {
	public:
		SimStatePlugin() :
			flow_nh("~sim_state"),
			uas(nullptr)
		{ };

		void initialize(UAS &uas_)
		{
			uas = &uas_;

			flow_nh.param<std::string>("frame_id", frame_id, "sim_state");

			accel_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("accel", 10);
			velocity_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("velocity", 10);
			position_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("position", 10);
			attitude_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("attitude", 10);
			angular_velocity_pub = flow_nh.advertise<geometry_msgs::Vector3Stamped>("angular_velocity", 10);
			quaternion_pub = flow_nh.advertise<geometry_msgs::QuaternionStamped>("quaternion", 10);
		}

		const message_map get_rx_handlers() {
			return {
				       MESSAGE_HANDLER(MAVLINK_MSG_ID_SIM_STATE, &SimStatePlugin::handle_sim_state)
			};
		}

	private:
		ros::NodeHandle flow_nh;
		UAS *uas;

		std::string frame_id;

		ros::Publisher accel_pub;
		ros::Publisher velocity_pub;
		ros::Publisher position_pub;
		ros::Publisher attitude_pub;
		ros::Publisher angular_velocity_pub;
		ros::Publisher quaternion_pub;

		void handle_sim_state(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
			mavlink_sim_state_t state;
			mavlink_msg_sim_state_decode(msg, &state);

			//Acceleration
			auto accel_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			accel_msg->header.stamp = ros::Time::now();
			accel_msg->vector.x = state.xacc;
			accel_msg->vector.y = state.yacc;
			accel_msg->vector.z = state.zacc;
			accel_pub.publish(accel_msg);

			// Velocity
			auto velocity_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			velocity_msg->header.stamp = ros::Time::now();
			velocity_msg->vector.x = state.vn;
			velocity_msg->vector.y = state.ve;
			velocity_msg->vector.z = state.vd;
			velocity_pub.publish(velocity_msg);

			// Position
			auto position_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			position_msg->header.stamp = ros::Time::now();
			position_msg->vector.x = state.lat;
			position_msg->vector.y = state.lon;
			position_msg->vector.z = state.alt;
			position_pub.publish(position_msg);

			auto attitude_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			attitude_msg->header.stamp = ros::Time::now();
			attitude_msg->vector.x = state.roll;
			attitude_msg->vector.y = state.pitch;
			attitude_msg->vector.z = state.yaw;
			attitude_pub.publish(attitude_msg);

			auto angular_velocity_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
			angular_velocity_msg->header.stamp = ros::Time::now();
			angular_velocity_msg->vector.x = state.xgyro;
			angular_velocity_msg->vector.y = state.ygyro;
			angular_velocity_msg->vector.z = state.zgyro;
			angular_velocity_pub.publish(angular_velocity_msg);

			auto quaternion_msg = boost::make_shared<geometry_msgs::QuaternionStamped>();
			quaternion_msg->header.stamp = ros::Time::now();
			quaternion_msg->quaternion.w = state.q1;
			quaternion_msg->quaternion.x = state.q2;
			quaternion_msg->quaternion.y = state.q3;
			quaternion_msg->quaternion.z = state.q4;
			quaternion_pub.publish(quaternion_msg);

		}
	};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SimStatePlugin, mavplugin::MavRosPlugin)