
#include <ual_ecar_vehicle_controller/VehicleControllerLowLevel.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "vehicle_controller");

		VehicleControllerLowLevel  vehicle_controller;
		vehicle_controller.initialize();

		ros::Rate loop_rate(100); // f : 100Hz => T = 10 ms
		while (ros::ok())
		{
			ros::spinOnce();
			vehicle_controller.iterate();
			loop_rate.sleep();
		}

		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
