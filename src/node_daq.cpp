#include <iostream>
#include <ros/ros.h>
#include "ros/package.h"
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
    ros::Rate rate(5);
    ROS_INFO_STREAM("sensor begin start");

    bool flag=false;
    double value=10.0;
    int time_count=0;
    while (ros::ok()){
        time_count++;
        geometry_msgs::Wrench msg;
        if(time_count>5*10){
            time_count=0;
            value=-1*value;
            flag= true;
        }
        msg.force.x=value;
        msg.force.y=value;
        msg.force.z=value;
        msg.torque.x=0.0;
        msg.torque.y=0.0;
        msg.torque.z=0.0;
        if(!flag){
            msg.force.x=0.0;
            msg.force.y=0.0;
            msg.force.z=0.0;
        }
        pub.publish(msg);
        rate.sleep();
    }

    return 0;
}

//    int RandomNumber;
//    while(ros::ok()){
//        srand((unsigned)time(NULL));//time()用系统时间初始化种。为rand()生成不同的随机种子。
//        RandomNumber = rand() % 10-5;
//        geometry_msgs::Wrench msg;
//        msg.force.x=RandomNumber;
//        msg.force.y=RandomNumber;
//        msg.force.z=RandomNumber;
//        msg.torque.x=0.0;
//        msg.torque.y=0.0;
//        msg.torque.z=0.0;
//        pub.publish(msg);
//        cout<<RandomNumber<<endl;
//        rate.sleep();
//    }
