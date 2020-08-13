
#ifndef FORCE_BRIDGE_FORCESERVICE_H
#define FORCE_BRIDGE_FORCESERVICE_H

#include <iostream>
#include "ros/ros.h"
#include "force/forcePluginAggre.h"
#include "matrixUtily.h"
#include "moveit/move_group_interface/move_group_interface.h"
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "vector"
#include "string"
#include "atomic"
#include "memory"
#include <eigen_conversions/eigen_msg.h>
#include <tf/LinearMath/Quaternion.h>

#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Int16.h"
#include "industrial_msgs/RobotStatus.h"

using namespace std;


struct MoveGroup{
    moveit::planning_interface::MoveGroupInterface *move_group;
    robot_model::RobotModelConstPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
    std::vector<std::string> joint_names;
    std::string groupName;
    string endlinkName;
};

class forceService {
public:
    explicit forceService(ros::NodeHandle* n);
    ~forceService();
    int start();

private:
    forcePluginAggre *force_plugin;
    MoveGroup MG;

    bool isSim ;
    bool is_stop ;
    bool is_running ;
    vector<double> currentForce, bias_force;

//    vector<double> startPos;
    double SA, SB, SC;
    bool XZReverseDirect;
    geometry_msgs::Pose SPose;
    atomic<bool> getForce,startPosOK ,robotStartStatus,robot_servo_status,ready_exit;
    //ros变量
    ros::NodeHandle* Node;
    ros::Publisher  impenderrResult, joint_state_pub;
    ros::ServiceServer impedenceStart_server;
    ros::Subscriber ready_stop_sub, start_pos_sub, force_sub, robot_status_sub;
//    ros::ServiceServer serverControl;

public:

    /***
     * 阻抗控制启动－－回调函数
     * @param req
     * @param res
     * @return
     */
    bool impedanceStartCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    /***
     * 初始化参数
     */
    int initParam();

    /***
     * 开启阻抗控制
     * @return
     */
    int StartImpedenceCtl();

    /***
     * 停止阻抗控制
     * @return
     */
    int StopImpedenceCtl();

    /***
     * 读取本地参数
     * @return
     */
    int readLocalParam();

    /***
     * 更新阻抗参数
     * @return
     */
    int updateParam();

    /***
     * 设置阻抗算法的输入传感器来源
     * @return
     */
    int setForceInputTopic();

    /***
     * 设置阻抗控制的方向
     * @return
     */
    int setForceControlDirect();

    //运动学模型加载初始化
    bool initKinematic();

    void forceCallbackZX(const geometry_msgs::Wrench::ConstPtr& msg);

    void forceCallbackXZ(const geometry_msgs::Wrench::ConstPtr& msg);

    void publishPose(std::vector<double> &joint_Pose);

    void getCurrentPose(geometry_msgs::Pose& Pose);

    int computeImpedence( std::vector<double> &force , std::vector<double> &outJoint);

    void robotStausCallback(const industrial_msgs::RobotStatusConstPtr& msg);
};


#endif //FORCE_BRIDGE_FORCESERVICE_H
