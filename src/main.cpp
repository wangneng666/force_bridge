#include <ros/ros.h>
#include "forceService.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "force_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(4);
    as.start();

    forceService fb(n);
    if(fb.start()!=0){
        return -1;
    }
    ros::waitForShutdown();
    return 0;
}

