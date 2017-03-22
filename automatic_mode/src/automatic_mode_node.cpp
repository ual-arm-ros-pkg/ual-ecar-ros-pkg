
#include <automatic_mode/AutomaticMode.h>
#include <std_msgs/String.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "automatic_mode");

		AutomaticMode  automatic_mode;
		automatic_mode.initialize();

		ros::Rate loop_rate(20);
		while (ros::ok())
		{
			ros::spinOnce();
			automatic_mode.iterate();
			loop_rate.sleep();
		}

		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
