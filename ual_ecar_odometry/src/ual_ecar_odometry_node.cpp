#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <phidgets_high_speed_encoder/EncoderDecimatedSpeed.h>

ros::Time last_encoder_pos_time;
double last_encoder_pos[2] = {0.0, 0.0};

ros::Time last_encoder_vel_time;
double last_encoder_vel[2] = {0.0, 0.0};

void onNewEncoderState(const sensor_msgs::JointState::Ptr& msg)
{
	ROS_INFO(
		"Received encoder pos: [0]=%12f [1]=%12f", msg->position[0],
		msg->position[1]);

	last_encoder_pos_time = msg->header.stamp;
	last_encoder_pos[0] = msg->position[0];
	last_encoder_pos[1] = msg->position[1];
}

void onNewEncoderSpeed(
	const phidgets_high_speed_encoder::EncoderDecimatedSpeed::ConstPtr& msg,
	int index)
{
	ROS_ASSERT(index < 2);
	last_encoder_vel_time = msg->header.stamp;
	ROS_INFO("Received encoder avr_speed: [%d]=%12f", index, msg->avr_speed);

	last_encoder_vel[index] = msg->avr_speed;
}

int main(int argc, char** argv)
{
	try
	{
		ros::init(argc, argv, "ual_ecar_odometry");

		ros::NodeHandle n;
		ros::NodeHandle nh_params = ros::NodeHandle("~");

		// now() must be called after Nodehandle
		last_encoder_vel_time = last_encoder_pos_time = ros::Time::now();

		ros::Subscriber sub_encoders =
			n.subscribe("joint_states", 10, &onNewEncoderState);
		ros::Subscriber sub_enc_decim_speed[2] = {
			n.subscribe<phidgets_high_speed_encoder::EncoderDecimatedSpeed>(
				"joint_states_ch0_decim_speed", 10,
				boost::bind(&onNewEncoderSpeed, _1, 0)),
			n.subscribe<phidgets_high_speed_encoder::EncoderDecimatedSpeed>(
				"joint_states_ch1_decim_speed", 10,
				boost::bind(&onNewEncoderSpeed, _1, 1))};

		ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
		tf::TransformBroadcaster odom_broadcaster;

		double x = 0.0;
		double y = 0.0;
		double th = 0.0;

		double vx = 0.1;
		double vy = -0.1;
		double vth = 0.1;

		ros::Time current_time, last_time;
		current_time = ros::Time::now();
		last_time = ros::Time::now();

		ros::Rate r(10.0);
		while (n.ok())
		{
			ros::spinOnce();  // check for incoming messages
			current_time = ros::Time::now();

			// compute odometry in a typical way given the velocities of the
			// robot
			double dt = (current_time - last_time).toSec();
			double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
			double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
			double delta_th = vth * dt;

			x += delta_x;
			y += delta_y;
			th += delta_th;

			// since all odometry is 6DOF we'll need a quaternion created from
			// yaw
			geometry_msgs::Quaternion odom_quat =
				tf::createQuaternionMsgFromYaw(th);

			// first, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			// send the transform
			odom_broadcaster.sendTransform(odom_trans);

			// next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "odom";

			// set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			// set the velocity
			odom.child_frame_id = "base_link";
			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.angular.z = vth;

			// publish the message
			odom_pub.publish(odom);

			last_time = current_time;
			r.sleep();
		}
	}
	catch (std::exception& e)
	{
		ROS_ERROR("Exception: %s", e.what());
	}
}
