#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Wrench.h"
#include "random"
using namespace std;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "node_daq");
    ros::NodeHandle node;
    ros::Publisher pub;
    pub=node.advertise<geometry_msgs::Wrench>("daq_data",1);
    ros::spinOnce();
    ros::Rate rate(50);
    ROS_INFO_STREAM("sensor begin start");
    int RandomNumber;
    while(ros::ok()){
        srand((unsigned)time(NULL));//time()用系统时间初始化种。为rand()生成不同的随机种子。
        RandomNumber = rand() % 10 + 1;
        geometry_msgs::Wrench msg;
        msg.force.x=RandomNumber/100.0;
        msg.force.y=RandomNumber/100.0;
        msg.force.z=RandomNumber/100.0;
        msg.torque.x=0.0;
        msg.torque.y=0.0;
        msg.torque.z=0.0;
        pub.publish(msg);
//        cout<<RandomNumber<<endl;
        rate.sleep();
    }
    return 0;
}

