/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2018  University of Almeria                               |
   +---------------------------------------------------------------------------+
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <phidgets_msgs/EncoderDecimatedSpeed.h>
#include <mrpt_bridge/time.h>  // ros2mrpt bridge
#include <mutex>
#include <array>
#include <cmath>
#include <string>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>
using mrpt::io::CFileGZOutputStream;
#else
#include <mrpt/utils/CFileGZOutputStream.h>
using mrpt::utils::CFileGZOutputStream;
#endif

class OdometryNode
{
   private:
	ros::NodeHandle n_, nh_params_;
	ros::Subscriber sub_encoders_, sub_enc_decim_speed_[2];
	ros::Publisher odom_pub_;
	tf::TransformBroadcaster odom_broadcaster;

	// Node variables & data types:
	struct EncoderPos
	{
		ros::Time timestamp;
		std::array<double, 2> pos{{0.0, 0.0}};  // tick counts
	};
	std::mutex new_enc_pos_mx_;
	EncoderPos last_enc_pos_, new_enc_pos_;
	bool first_enc_pos_{true};
	ros::Time last_encoder_vel_time_;
	std::array<double, 2> last_encoder_vel = {0.0, 0.0};
	std::mutex last_encoder_vel_mx;

	// Node parameters:
	double ODOM_PUBLISH_RATE_{5.0};  // Hz
	std::string rawlog_asf_prefix_;  // If not empty, generate action-SF rawlog
	std::string rawlog_obs_prefix_;  // If not empty, generate obs-only rawlog
	double left_K_{3.1e-5}, right_K_{-1.22e-4}, wheels_dist_{1.26};

	mrpt::math::TPose2D global_odometry_{0, 0, 0};
	mrpt::math::TTwist2D cur_vel_{0, 0, 0};

	CFileGZOutputStream out_rawlog_obs_, out_rawlog_actsf_;

	// callback for topic /joint_states
	void onNewEncoderState(const sensor_msgs::JointState::Ptr& msg)
	{
		ROS_DEBUG(
			"Received encoder pos: [0]=%12f [1]=%12f", msg->position[0],
			msg->position[1]);

		std::lock_guard<std::mutex> lk(new_enc_pos_mx_);

		new_enc_pos_.timestamp = msg->header.stamp;
		new_enc_pos_.pos[0] = msg->position[0];
		new_enc_pos_.pos[1] = msg->position[1];
	}

	// callback for topics /joint_states_ch{0,1}_decim_speed
	void onNewEncoderSpeed(
		const phidgets_msgs::EncoderDecimatedSpeed::ConstPtr& msg,
		int index)
	{
		std::lock_guard<std::mutex> lk(last_encoder_vel_mx);

		ROS_ASSERT(index < 2);
		last_encoder_vel_time_ = msg->header.stamp;
		ROS_DEBUG(
			"Received encoder avr_speed: [%d]=%12f", index, msg->avr_speed);

		last_encoder_vel[index] = msg->avr_speed;
	}

	/** The odometry model: converts encoder ticks to pose increments, using
	 * the parameter loaded from the config file */
	void differentialOdometryModel(
		mrpt::math::TPose2D& out_increment, double left_tick_incr,
		double right_tick_incr) const
	{
		double As =
			0.5 * (right_K_ * right_tick_incr + left_K_ * left_tick_incr);
		double Aphi = (right_K_ * right_tick_incr - left_K_ * left_tick_incr) /
					  wheels_dist_;
		//    cout << "As_left = " << left_K_*left_tick_incr << " ticks: " <<
		//    left_tick_incr << endl; cout << "As_right = " <<
		//    right_K_*right_tick_incr << " ticks: " << right_tick_incr <<
		//    endl;

		out_increment.x = cos(Aphi) * As;
		out_increment.y = sin(Aphi) * As;
		out_increment.phi = Aphi;
	}

	/** The odometry model in velocity: converts encoder velocities to vehicle
	 * velocities, using the parameter loaded from the config file */
	void differentialOdometryModelVel(
		double& out_v, double& out_w, const double left_ticks_vel,
		const double right_ticks_vel) const
	{
		out_v = 0.5 * (right_K_ * right_ticks_vel + left_K_ * left_ticks_vel);
		out_w = (right_K_ * right_ticks_vel - left_K_ * left_ticks_vel) /
				wheels_dist_;
	}

   public:
	// ctor:
	OdometryNode() : n_(), nh_params_{ros::NodeHandle("~")}
	{
		// Nothing else to do
	}

	void init()
	{
		// Node parameters:
		nh_params_.getParam("ODOM_PUBLISH_RATE", ODOM_PUBLISH_RATE_);
		nh_params_.getParam("rawlog_asf_prefix", rawlog_asf_prefix_);
		nh_params_.getParam("rawlog_obs_prefix", rawlog_obs_prefix_);
		nh_params_.getParam("left_K", left_K_);
		nh_params_.getParam("right_K", right_K_);
		nh_params_.getParam("wheels_dist", wheels_dist_);

		// now() must be called after Nodehandle
		last_encoder_vel_time_ = new_enc_pos_.timestamp =
			last_enc_pos_.timestamp = ros::Time::now();

		sub_encoders_ = n_.subscribe(
			"joint_states", 10, &OdometryNode::onNewEncoderState, this);
		sub_enc_decim_speed_[0] =
			n_.subscribe<phidgets_msgs::EncoderDecimatedSpeed>(
				"joint_states_ch0_decim_speed", 10,
				boost::bind(&OdometryNode::onNewEncoderSpeed, this, _1, 0));
		sub_enc_decim_speed_[1] =
			n_.subscribe<phidgets_msgs::EncoderDecimatedSpeed>(
				"joint_states_ch1_decim_speed", 10,
				boost::bind(&OdometryNode::onNewEncoderSpeed, this, _1, 1));

		odom_pub_ = n_.advertise<nav_msgs::Odometry>("odom", 50);

		// Open rawlog files:
		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(mrpt::system::now(), parts, true);
		std::string rawlog_postfix = mrpt::format(
			"_%04u-%02u-%02u_%02uh%02um%02us.rawlog", (unsigned int)parts.year,
			(unsigned int)parts.month, (unsigned int)parts.day,
			(unsigned int)parts.hour, (unsigned int)parts.minute,
			(unsigned int)parts.second);
		rawlog_postfix =
			mrpt::system::fileNameStripInvalidChars(rawlog_postfix);

		if (!rawlog_obs_prefix_.empty())
		{
			std::string fil = rawlog_obs_prefix_ + rawlog_postfix;
			ROS_INFO("Writing rawlog (obs-only format) to: %s", fil.c_str());
			out_rawlog_obs_.open(fil);
		}
		if (!rawlog_asf_prefix_.empty())
		{
			std::string fil = rawlog_asf_prefix_ + rawlog_postfix;
			ROS_INFO("Writing rawlog (act-sf format) to: %s", fil.c_str());
			out_rawlog_actsf_.open(fil);
		}

	}  // end init()

	void run()
	{
		// static data for TF odom:
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		ros::Time last_processed_encoder_vel_time_;

		ros::Rate r(ODOM_PUBLISH_RATE_);
		while (n_.ok())
		{
			// check for incoming messages. Single threaded model:
			ros::spinOnce();

			// new encoder position data?
			new_enc_pos_mx_.lock();

			if (new_enc_pos_.timestamp != last_enc_pos_.timestamp)
			{
				// Yes: process new encoder positions:
				MRPT_TODO("Handle potential encoders overflow!!");

				// Compute increment:
				double left_incr = new_enc_pos_.pos[0] - last_enc_pos_.pos[0];
				double right_incr = new_enc_pos_.pos[1] - last_enc_pos_.pos[1];

				// if we are in the first read, discard the increment and start
				// from the current count values:
				if (first_enc_pos_)
				{
					first_enc_pos_ = false;
					left_incr = right_incr = 0;
				}

				// Store new as old:
				last_enc_pos_ = new_enc_pos_;

				// kinematic model:
				mrpt::math::TPose2D odo_incr;
				this->differentialOdometryModel(
					odo_incr, left_incr, right_incr);
				ROS_INFO(
					"incrs: (%f,%f) -> %s", left_incr, right_incr,
					odo_incr.asString().c_str());

				// Cummulative odometry:
				global_odometry_ = global_odometry_ + odo_incr;
				std::string sOdo;
				global_odometry_.asString(sOdo);
				ROS_DEBUG_STREAM("New odometry: " << sOdo);

				// publish odometry as ROS tf: odom -> base_link
				{
					odom_trans.header.stamp = new_enc_pos_.timestamp;
					odom_trans.transform.translation.x = global_odometry_.x;
					odom_trans.transform.translation.y = global_odometry_.y;
					odom_trans.transform.translation.z = 0.0;
					odom_trans.transform.rotation =
						tf::createQuaternionMsgFromYaw(global_odometry_.phi);

					odom_broadcaster.sendTransform(odom_trans);
				}

				// if enabled, save to obs-only rawlog:
				if (out_rawlog_obs_.is_open())
				{
					auto odom = mrpt::obs::CObservationOdometry::Create();
					mrpt_bridge::convert(
						new_enc_pos_.timestamp, odom->timestamp);
					odom->sensorLabel = "ODOMETRY";
					odom->odometry = mrpt::poses::CPose2D(global_odometry_);
					odom->hasVelocities = true;
					odom->velocityLocal = cur_vel_;
					odom->hasEncodersInfo = true;
					odom->encoderLeftTicks = left_incr;
					odom->encoderRightTicks = right_incr;

					// Serialize:
#if MRPT_VERSION >= 0x199
					auto arch =
						mrpt::serialization::archiveFrom(out_rawlog_obs_);
#else
					auto& arch = out_rawlog_obs_;
#endif
					arch << odom;
				}
			}  // end new odom pos data

			new_enc_pos_mx_.unlock();

			// Process new velocity:
			if (last_processed_encoder_vel_time_ != last_encoder_vel_time_)
			{
				std::lock_guard<std::mutex> lk(last_encoder_vel_mx);

				last_processed_encoder_vel_time_ = last_encoder_vel_time_;

				const double vel_left = last_encoder_vel[0],
							 vel_right = last_encoder_vel[1];

				cur_vel_.vy = 0;
				differentialOdometryModelVel(
					cur_vel_.vx, cur_vel_.omega, vel_left, vel_right);
			}
			else
			{
				MRPT_TODO("reset vel to zero if no news");
			}

			// publish odom topic:
			{
				nav_msgs::Odometry odom;
				odom.header.stamp = new_enc_pos_.timestamp;
				odom.header.frame_id = "odom";

				// set the position
				odom.pose.pose.position.x = global_odometry_.x;
				odom.pose.pose.position.y = global_odometry_.y;
				odom.pose.pose.position.z = 0.0;
				odom.pose.pose.orientation =
					tf::createQuaternionMsgFromYaw(global_odometry_.phi);

				// set the velocity
				odom.child_frame_id = "base_link";
				odom.twist.twist.linear.x = cur_vel_.vx;
				odom.twist.twist.linear.y = cur_vel_.vy;
				odom.twist.twist.angular.z = cur_vel_.omega;

				// publish the message
				odom_pub_.publish(odom);
			}

			r.sleep();
		}
	}  // end run()

};  // end class OdometryNode

int main(int argc, char** argv)
{
	try
	{
		ros::init(argc, argv, "ual_ecar_odometry");

		OdometryNode node;

		node.init();
		node.run();
	}
	catch (std::exception& e)
	{
		ROS_ERROR("Exception: %s", e.what());
	}
}
