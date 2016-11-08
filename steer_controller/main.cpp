/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include "CSteerControllerLowLevel.h"

int main(int argc ,char * argv[])
{
    const char * sMissionFile = "Mission.moos";
    const char * sMOOSName = "SteerControllerLowLevel";
    switch(argc)
    {
    case 3:
        sMOOSName = argv[2];
    case 2:
        sMissionFile = argv[1];
    }

    CSteerControllerLowLevel TheApp;
    TheApp.Run(sMOOSName,sMissionFile);
    return 0;
}
