#include <iostream>
#include "ros/ros.h"
#include "hirop_msgs/shakeHandSet.h"
#include "hirop_msgs/shakeHandStatus.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "atomic"
using namespace std;

struct shakeHand{
    bool recordInitPose= false; //记录初始标志
    double initRobotPose[3]{};
    double CurRobotPose[3]{};
    double before_force[3]{};
    double currentForce[3]{};
    double MaxDistance{};
    double MinDistance{};
    int countTime{};
    atomic<int > shakeStatus{};
    int timer_noMove{};
    int shakeCount{};
};

//全局变量
shakeHand sh_data; //controller para
int step;//0:等待握手检测，1:进行握手检测
bool shakeHand_begin=false;
bool shakeHand_end= false;
ros::Publisher shakeHandStatus_pub;//发布握手状态


//服务回调函数 0.04,0.005,3
bool shakeHand_beginCB(hirop_msgs::shakeHandSet::Request &req, hirop_msgs::shakeHandSet::Response &res){
    sh_data.MaxDistance=req.MaxDistance;
    sh_data.MinDistance=req.MinDistance;
    sh_data.countTime=req.countTime;
    shakeHand_begin=true;
    shakeHand_end= false;
    int time_count=0;
    while (step!=1){
        usleep(100*1000);
        time_count++;
        if(time_count>100){
            break;
        }
    }
    //话题与服务初始化
    res.is_success=true;
    return true;
}

bool shakeHand_endCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    shakeHand_end=true;
    shakeHand_begin= false;
    res.success=true;
    return true;
}

double cul_forceDiff(){
    double tmp_diffForce=0.0;
    for (int i = 0; i <3; ++i) {
        tmp_diffForce+=pow((fabs(sh_data.before_force[i]-sh_data.currentForce[i])),2);
    }
    return tmp_diffForce;
}

void calculate_shakeHand(){
    //只进行X与Z方向判断
//    double tmp_Distance = sqrt(pow((fabs(sh_data.initRobotPose[0]-sh_data.CurRobotPose[0])),2) +pow((fabs(sh_data.initRobotPose[2]-sh_data.CurRobotPose[2])),2));
    //只进行Z方向判断
    double tmp_Distance = sqrt( pow((fabs(sh_data.initRobotPose[2]-sh_data.CurRobotPose[2])),2));
    std::cout << "<------------ cal the distance is "<< tmp_Distance<<std::endl;
//    ROS_INFO_STREAM("<------------  shakeStatus is "<< sh_data.shakeStatus<< " sh_data.countTime is "<<sh_data.countTime);

    // no force control
    if( cul_forceDiff()< 0.003){
        sh_data.timer_noMove++;
    }else{
        sh_data.timer_noMove = 0;
    }

    //to max
    if((tmp_Distance>sh_data.MaxDistance )&& (sh_data.shakeStatus == 0)){
        sh_data.shakeStatus = 1;
    }

    // back distance
    if ((tmp_Distance < sh_data.MinDistance )&&( sh_data.shakeStatus  == 1) ){
        ROS_INFO_STREAM("true or false"<<( sh_data.shakeStatus  == 1));
        sh_data.shakeStatus = 0;
        sh_data.shakeCount++;
        cout<<"摆动一次"<<endl;

    }

    //等待退出
    if( sh_data.shakeCount>=sh_data.countTime){
        ROS_INFO_STREAM("<------------  waitForExit "<<"-----timer_noMove "<<sh_data.timer_noMove);
        //等待没有力矩感应
        if((sh_data.currentForce[0]<0.1)&&(sh_data.currentForce[1]<0.1)&&(sh_data.currentForce[2]<0.1)){
            sh_data.timer_noMove++;//计时
        } else{
            sh_data.timer_noMove=0;
        }
    }
    //发布握手完成信号（超时）或（达到晃动次数要求与无力矩感应）
    if(sh_data.timer_noMove > 500 ||((sh_data.timer_noMove > 50)&&(sh_data.shakeCount>=sh_data.countTime))){
        hirop_msgs::shakeHandStatus msg;
        msg.shakeHand_count=sh_data.shakeCount;
        msg.shakeHand_over=true;
        shakeHandStatus_pub.publish(msg);
    }
    ROS_INFO_STREAM("<------shakeStatus "<< sh_data.shakeStatus<<"----shakeCount "<<sh_data.shakeCount);
}


void resetData(){
    sh_data.shakeStatus=0;
    sh_data.timer_noMove=0;
    sh_data.recordInitPose=false;
    sh_data.countTime=0;
    sh_data.shakeCount=0;
    shakeHand_begin= false;
    shakeHand_end= false;
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



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "shakeHandJudge");
    ros::NodeHandle node;
    ros::AsyncSpinner as(2);
    as.start();
    //初始化变量
    ros::Rate rate(10);//10hz
    step=0;
    //初始化服务与话题
    //设置最大距离，最小距离，握手摆动次数
    ros::ServiceServer bringUp_server = node.advertiseService("shakeHandJudge/begin", shakeHand_beginCB);
    //关闭握手服务
    ros::ServiceServer shutdown_server = node.advertiseService("shakeHandJudge/end", shakeHand_endCB);
    shakeHandStatus_pub= node.advertise<hirop_msgs::shakeHandStatus>("shakeHandJudge/Status", 1);
    //力矩传感器数据接收
    ros::Subscriber  forceSensor_sub=node.subscribe("/daq_data", 1, forceSensorCB);
    //机器人坐标点接收
    ros::Subscriber robot_XYZPose_sub=node.subscribe("force_bridge/robotPose", 1,robotPose_XYZ_CB);
    ROS_INFO_STREAM("shakeHandJudge init over");
    while(ros::ok()&&(!ros::isShuttingDown()))
    {
        //等待启动握手检测服务
        if(step==0)
        {
            if(shakeHand_begin){
                step=1;
            }
        }
        //进行握手检测
        if(step==1)
        {
            calculate_shakeHand();
            if(shakeHand_end){
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
    ROS_INFO_STREAM("shakeHandJudge exit   ");
    return 0;
}