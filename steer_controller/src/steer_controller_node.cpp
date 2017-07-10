
#include <steer_controller/CSteerControllerLowLevel.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "steer_controller");

		CSteerControllerLowLevel  steer_controller;
		steer_controller.initialize();

		ros::Rate loop_rate(50); /*50ms es el tiempo tomado para el diseño de los controladores*/
		while (ros::ok())
		{
			ros::spinOnce();
			steer_controller.iterate();
			loop_rate.sleep();
		}

		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
