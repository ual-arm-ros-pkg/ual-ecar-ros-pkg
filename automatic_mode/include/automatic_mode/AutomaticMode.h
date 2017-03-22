/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

   #pragma once

   #include <ros/ros.h>
//   #include <_____.h>
   #include <std_msgs/Bool.h>
   #include <std_msgs/Float64.h>

   class AutomaticMode {
   public:
   	/**
   	* NodeHandle is the main access point to communications with the ROS system.
   	* The first NodeHandle constructed will fully initialize this node, and the last
   	* NodeHandle destructed will close down the node.
   	*/
   	ros::NodeHandle m_nh = ros::NodeHandle();
   	ros::NodeHandle m_nh_params = ros::NodeHandle("~");

//  ros::Subscriber m_sub_
   	ros::Publisher  m_pub_mode;

   	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
   	  * \return false on error */
   	bool initialize();

   	/** called when work is to be done */
   	bool iterate();

   };
