
#include <system_identification/SystemIdentification.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, "system_identification");

		SystemIdentification  system_identification;
		system_identification.initialize();

		ros::Rate loop_rate(50); /*50ms es el tiempo tomado para el diseño de los controladores*/
		while (ros::ok())
		{
			ros::spinOnce();
			system_identification.iterate();
			loop_rate.sleep();
		}

		return 0;

	} catch (std::exception &e)
	{
		ROS_ERROR("Exception: %s",e.what());
	}
}
