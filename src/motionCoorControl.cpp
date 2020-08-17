#include <iostream>
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

using namespace std;




//服务回调函数 //设置阻抗随动方向
bool motionCoorSetCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    res.success=true;
    return true;
}

bool motionCoorShutDownCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    res.success=true;
    return true;
}





int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motionCoorControl");
    ros::NodeHandle node;
    ros::AsyncSpinner as(2);
    as.start();
    //初始化服务与话题
    ros::ServiceServer motionCoorSet_server = node.advertiseService("motionCoorControl_set", motionCoorSetCB);
    ros::ServiceServer motionCoorShutDown_server = node.advertiseService("motionCoorControl_shutdown", motionCoorShutDownCB);
    ros::waitForShutdown();
    return 0;
}