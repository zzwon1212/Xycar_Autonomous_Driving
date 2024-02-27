#include "autonomous_driving/driving.h"

using PREC = float;

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "Driving");
    xycar::Driving<PREC> Driving;
    Driving.run();

    return 0;
}
