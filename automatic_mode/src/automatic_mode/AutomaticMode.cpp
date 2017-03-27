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

<<<<<<< HEAD
  // Try to connect...
	if (this->AttemptConnection())
	{
    ROS_INFO("Connection OK to automatic mode."); 
=======
  // Default value:
  // Se haría así la inicialización del modo de conducción o como está en las líneas 33-37?
  bool ecar_joystick_mode=true;
  m_nh_params.getParam("ECAR_JOYSTICK_MODE",ecar_joystick_mode);

  // Try to connect...
	if (this->AttemptConnection())
	{
//  ROS_INFO("Connection OK to automatic mode."); // Esto debería salir solo si se pasa a automático, no?
>>>>>>> refs/remotes/ual-arm-ros-pkg/master
    CMD_SetAutomaticMode(ecar_joystick_mode);

		// TODO: initial controller parameters
	}
  m_pub_mode= m_nh.advertise<std_msgs::Bool>("automatic_mode", 10);
<<<<<<< HEAD
    {
  		std_msgs::Bool b;
  		b.data = false;
  		m_pub_mode.publish(b);
    }
=======
/*{
  		std_msgs::Bool b;
  		b.data = true;
  		m_pub_rev_relay.publish(b);
  }*/
>>>>>>> refs/remotes/ual-arm-ros-pkg/master
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
