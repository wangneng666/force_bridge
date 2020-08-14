#include <ros/ros.h>
#include "forceService.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "force_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(1);
    as.start();

    forceService fb(&n);
    fb.start();
    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();
//    ros::waitForShutdown();
    return 0;
}

