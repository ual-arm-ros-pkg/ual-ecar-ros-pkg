/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <steer_controller/CSteerControllerLowLevel.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;



/** Convert encoder ticks into Ackermann central angle (radians) */
double CSteerControllerLowLevel::ackermannAngle(int32_t ticks) const
{
	MRPT_TODO("*** Put Ackermann model here!!!! *** ")
	return 1.0*ticks;
}


/** Convert Ackermann central angle (radians) to encoder ticks: */
int32_t CSteerControllerLowLevel::ackermannAngle_inverse(double angle_radians) const
{
	MRPT_TODO("*** Put inverse Ackermann model here!!!! *** ")
	return angle_radians;
}
