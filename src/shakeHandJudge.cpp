#include <iostream>
#include "ros/ros.h"
#include "hirop_msgs/shakeHandSet.h"
#include "hirop_msgs/shakeHandStatus.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
using namespace std;

struct shakeHand{
    bool recordInitPose= false; //记录初始标志
    double initRobotPose[3]{};
    double CurRobotPose[3]{};
    double currentForce[3]{};
    double MaxDistance{};
    double MinDistance{};
    int countTime{};
    int shakeStatus{};
    int noMove{};
    int shakeCount{};
};

//全局变量
shakeHand sh_data; //controller para
int step;//0:等待握手检测，1:进行握手检测
bool shakeHand_bringUp=false;
bool shakeHand_shutDown= false;
ros::ServiceServer bringUp_server;//设置最大距离，最小距离，握手摆动次数
ros::ServiceServer shutdown_server;//关闭握手服务
ros::Publisher shakeHandStatus_pub;//发布握手状态
ros::Subscriber forceSensor_sub; //力矩传感器数据接收
ros::Subscriber robot_XYZPose_sub;   //机器人坐标点接收

//服务回调函数
bool shakeHand_bringUpCB(hirop_msgs::shakeHandSet::Request &req, hirop_msgs::shakeHandSet::Response &res){
    sh_data.MinDistance=req.MinDistance;
    sh_data.MinDistance=req.MinDistance;
    sh_data.countTime=req.countTime;
    shakeHand_bringUp=true;
    shakeHand_shutDown= false;
    while (step!=1){
        usleep(100*1000);
    }
    //话题与服务初始化
    res.is_success=true;
    return true;
}

bool shakeHand_shutDownCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    shakeHand_shutDown=true;
    shakeHand_bringUp= false;
    res.success=true;
    return true;
}

void forceSensorCB(const geometry_msgs::Wrench_<allocator<void>>::ConstPtr &msg){
    sh_data.currentForce[0]=msg->force.x;
    sh_data.currentForce[1]=msg->force.y;
    sh_data.currentForce[2]=msg->force.z;
}

void robotPose_XYZ_CB(const geometry_msgs::Pose::ConstPtr &msg){
    sh_data.CurRobotPose[0]=msg->position.x;
    sh_data.CurRobotPose[1]=msg->position.y;
    sh_data.CurRobotPose[2]=msg->position.z;
    if(!sh_data.recordInitPose){
        sh_data.initRobotPose[0]=msg->position.x;
        sh_data.initRobotPose[1]=msg->position.y;
        sh_data.initRobotPose[2]=msg->position.z;
        sh_data.recordInitPose= true;
    }
}

void calculate_shakeHand(){
    //只进行X与Z方向判断
    double tmp_Distance = sqrt(pow((fabs(sh_data.initRobotPose[0]-sh_data.CurRobotPose[0])),2) +pow((fabs(sh_data.initRobotPose[2]-sh_data.CurRobotPose[2])),2));
    std::cout << "<------------ cal the distance is "<< tmp_Distance<<std::endl;
    ROS_INFO_STREAM("<------------  shakeStatus is "<< sh_data.shakeStatus<< " sh_data.countTime is "<<sh_data.countTime);

    // no force control
    if(tmp_Distance< 0.003){
        sh_data.noMove++;
    }else{
        sh_data.noMove = 0;
    }

    //to max
    if(tmp_Distance>sh_data.MaxDistance && sh_data.shakeStatus == 0){
        sh_data.shakeStatus = 1;
    }

    // back distance
    if (tmp_Distance < sh_data.MinDistance && sh_data.shakeStatus  == 1 ){
        sh_data.shakeStatus = 0;
        sh_data.shakeCount++;
    }

    //等待退出
    if( sh_data.shakeCount>=sh_data.countTime){
        ROS_INFO_STREAM("<------------  waitForExit "<<"-----noMove "<<sh_data.noMove);
        //等待没有力矩感应
        if((sh_data.currentForce[0]<0.1)&&(sh_data.currentForce[1]<0.1)&&(sh_data.currentForce[2]<0.1)){
            sh_data.noMove++;//计时
        } else{
            sh_data.noMove=0;
        }
    }
    //发布握手完成信号（超时）或（达到晃动次数要求与无力矩感应）
    if(sh_data.noMove > 500 ||((sh_data.noMove > 50)&&(sh_data.shakeCount>=sh_data.countTime))){
        hirop_msgs::shakeHandStatus msg;
        msg.shakeHand_count=sh_data.shakeCount;
        msg.shakeHand_over=true;
        shakeHandStatus_pub.publish(msg);
    }
    ROS_INFO_STREAM("<----------shakeStatus is "<< sh_data.shakeStatus<<"----shakeCount is "<<sh_data.shakeCount);
}



void resetData(){
    sh_data.shakeStatus=0;
    sh_data.noMove=0;
    sh_data.recordInitPose=false;
    sh_data.countTime=0;
    sh_data.shakeCount=0;
    shakeHand_bringUp= false;
    shakeHand_shutDown= false;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "shakeHandJudge");
    ros::NodeHandle node;
    ros::AsyncSpinner as(2);
    as.start();
    //初始化服务与话题
    bringUp_server = node.advertiseService("shakeHandJudge_bringUp", shakeHand_bringUpCB);
    shutdown_server = node.advertiseService("shakeHandJudge_shutdown", shakeHand_shutDownCB);
    shakeHandStatus_pub= node.advertise<hirop_msgs::shakeHandStatus>("shakeHandStatus", 1);
    forceSensor_sub=node.subscribe("forceSensor_data", 1, forceSensorCB);
    robot_XYZPose_sub=node.subscribe("robot_XYZPose", 1,robotPose_XYZ_CB);

    ros::Rate rate(10);
    step=0;
    while(ros::ok())
    {
        //等待启动握手检测服务
        if(step==0)
        {
            if(shakeHand_bringUp){
                step=1;
            }
        }
        //进行握手检测
        if(step==1)
        {
            calculate_shakeHand();
            if(shakeHand_shutDown){
                step=2;
            }
        }
        //退出复位
        if(step==2){
            resetData();
            step=0;
        }
        rate.sleep();
    }
    return 0;
}