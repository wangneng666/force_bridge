#include <iostream>
#include "ros/ros.h"
using namespace std;

struct shakeHand{
    bool recordInitPose= false; //记录初始标志
    double initRobotPose[3]{};
    double CurRobotPose[3]{};
    double initForce[3]{};
    double currentForce[3]{};
    double MaxDistance{};
    double MinDistance{};
    int countTime{};
};


class shakeHandJudge{
private:
    std::vector<double> forceData;		// 保存传感器数据
    std::vector<double> robPoseData;		// 保存robot数据
    shakeHand sh_data; //controller para
    bool downFlag, upFalg;
    int shakeStatus;
    int noMove;
    ros::ServiceServer bringUp_server;//设置最大距离，最小距离，握手摆动次数
    ros::ServiceServer shutdown_server;//关闭握手服务
    ros::Publisher shakeHandStatus_pub;//发布握手状态
    ros::Subscriber forceSensor_sub; //力矩传感器数据接收
    ros::Subscriber robotXYZPose_sub;   //机器人坐标点接收

public:
    shakeHandJudge(ros::NodeHandle Node){

    }

    bool impedanceStartCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);


public:
    //更新传感器数据
    void updateForceAndPosData(const vector<double> &_forceData,const vector<double> &_robPoseData){
        if(sh_data.recordInitPose == false){
            copy(_robPoseData.begin(), _robPoseData.begin()+ 3,sh_data.initRobotPose);
            copy(_forceData.begin(), _forceData.begin()+ 3,sh_data.initForce);

            sh_data.recordInitPose = true;
            return ;
        }

        copy(_robPoseData.begin(), _robPoseData.begin()+ 3,sh_data.CurRobotPose);
        copy(_forceData.begin(), _forceData.begin()+ 3,sh_data.initForce);
    }

    /***
     * 计算握手次数
     */
    void calculate_shakeHand(){

        double tmp_Distance = sqrt(pow((fabs(sh_data.initRobotPose[0]-sh_data.CurRobotPose[0])),2) +pow((fabs(sh_data.initRobotPose[2]-sh_data.CurRobotPose[2])),2));
        std::cout << "<------------ cal the distance is "<< tmp_Distance<<std::endl;
        ROS_INFO_STREAM("<------------  shakeStatus is "<< shakeStatus<< " sh_data.countTime is "<<sh_data.countTime);

        // no force control
        if(tmp_Distance< 0.003){
            noMove++;
        }else{
            noMove = 0;
        }

        //to max
        if(tmp_Distance>sh_data.MaxDistance && shakeStatus == 0){
            shakeStatus = 1;
        }

        // qi
        if(tmp_Distance > sh_data.MinDistance && shakeStatus == 1){
            shakeStatus = 2;
        }

        // back distance
        if (tmp_Distance < sh_data.MinDistance && shakeStatus  == 2 ){
            shakeStatus = 0;
            sh_data.countTime -=1;
            if(sh_data.countTime == 1)
                sh_data.MinDistance = 0.003;
        }

    }

    /***
     * 判断握手结束
     */
    bool getShakeHandStatus(){
        if( noMove > 500){
            ROS_ERROR_STREAM("<- noMove is  ok");
            return true;
        }
        if(sh_data.countTime <=0 ){
            ROS_ERROR_STREAM( "<- countTime is  ok");
            noMove = 0;
            return true;
        }
        return false;
    }

    bool waitForExit(){
        double tmp_Distance = sqrt(pow((fabs(sh_data.initRobotPose[0]-sh_data.CurRobotPose[0])),2) +pow((fabs(sh_data.initRobotPose[2]-sh_data.CurRobotPose[2])),2));
        ROS_INFO_STREAM("<------------  waitForExit is "<< noMove <<" "<<tmp_Distance);

        if(tmp_Distance< 0.003){
            noMove++;
        }

        if(noMove > 50){
            resetData();
            return true;
        }
        else
            return false;
    }
private:
    void resetData(){
        downFlag = false;
        upFalg = false;
        shakeStatus = 0;
        noMove = 0;
    }

};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "shakeHandJudge");
    ros::NodeHandle node;
    ros::AsyncSpinner as(2);
    as.start();
    ros::waitForShutdown();
    return 0;
}