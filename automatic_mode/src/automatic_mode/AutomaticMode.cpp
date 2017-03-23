/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <automatic_mode/AutomaticMode.h>
#include <ros/console.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

//bool AutomaticMode::AutomaticMode(){
//}

bool AutomaticMode::initialize(){

  // Try to connect...
	if (this->AttemptConnection())
	{
    ROS_INFO("Connection OK to automatic mode."); 
    CMD_SetAutomaticMode(ecar_joystick_mode);

		// TODO: initial controller parameters
	}
  m_pub_mode= m_nh.advertise<std_msgs::Bool>("automatic_mode", 10);
    {
  		std_msgs::Bool b;
  		b.data = false;
  		m_pub_mode.publish(b);
    }
}

bool AutomaticMode::iterate(){
  bool mode, button_mode;

// Lectura del botón del usuario para definición del modo
	bool ok = m_mode.getMode(0,button_mode);
  	if (!ok) {
    ROS_ERROR("AutomaticMode: Manual conduction");
    return false;
  }
  // Mode button:
	// ---------------
	const bool mode_btn = (mode);
	{
		std_msgs::Bool b;
		b.data = mode_btn;
		m_pub_mode.publish(b);
	}
  // Controller design

  return true;
}
