#include "autonomous_driving/driving.h"

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "Driving");
    xycar::Driving Driving;
    Driving.run();

    return 0;
}
