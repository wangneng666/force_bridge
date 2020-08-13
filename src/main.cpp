#include <ros/ros.h>
#include "forceService.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "force_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(2);
    as.start();

    forceService fb(&n);
    fb.start();

    ros::waitForShutdown();
    return 0;
}

