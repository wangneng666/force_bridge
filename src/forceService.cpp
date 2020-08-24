#include "forceService.h"
forceService::forceService(ros::NodeHandle* n) {
    Node = n;
    force_plugin = new forcePluginAggre();
}

forceService::~forceService() {
    if(force_plugin!= nullptr)
    {
        delete force_plugin;
        force_plugin = nullptr;
    }
}

int forceService::start() {
    //参数初始化
    if(initParam()!=0){
        return -1;
    }
    //moveit模型初始化
    if(!initKinematic()){
        return -2;
    }
    return 0;
}

//初始化参数
int forceService::initParam() {
    int ret=0;
    Node->getParam("/force_bridge/group_name",MG.groupName);
    Node->getParam("/force_bridge/endlinkName",MG.endlinkName);
    Node->getParam("/force_bridge/isSim",isSim);
    Node->getParam("/force_bridge/XZReverseDirect",XZReverseDirect);
    ROS_INFO_STREAM("groupName "<<MG.groupName);
    ROS_INFO_STREAM("endlinkName "<<MG.endlinkName);
    ROS_INFO_STREAM("isSim "<<isSim);
    ROS_INFO_STREAM("XZReverseDirect: "<<XZReverseDirect);
    if(readLocalParam()!=0){
        return -1;
    }

    //变量初始化
    flag_SetForceBias= false;
    bias_force.resize(6);
    std::fill(bias_force.begin(), bias_force.begin()+6 ,0);
    currentForce.resize(6);
    std::fill(currentForce.begin(), currentForce.begin()+6 ,0);
    //话题初始化
    Pose_state_pub = Node->advertise<geometry_msgs::Pose>("force_bridge/robotPose", 1);
    robot_status_sub=Node->subscribe<industrial_msgs::RobotStatus>("robot_status",1,boost::bind(&forceService::robotStausCallback,this,_1));

    if(isSim)
        joint_state_pub = Node->advertise<sensor_msgs::JointState>("joint_states", 1);
    else
        joint_state_pub = Node->advertise<sensor_msgs::JointState>("impedance_err", 1);

    if(!XZReverseDirect)
        force_sub = Node->subscribe("/daq_data", 1, &forceService::forceCallbackXZ, this);
    else
        force_sub = Node->subscribe("/daq_data", 1, &forceService::forceCallbackZX, this);

    force_algorithmChange_server = Node->advertiseService("force_bridge/force_algorithm", &forceService::force_algorithmChangeCB,this);
    impedenceStart_server = Node->advertiseService("force_bridge/impedenceStart", &forceService::impedenceStartCB,this);
    impedenceClose_server = Node->advertiseService("force_bridge/impedenceClose", &forceService::impedenceCloseCB,this);
    ROS_INFO_STREAM("--------initParam over------------");

    return ret;

}

//开启阻抗控制
int forceService::StartImpedenceCtl() {
    ROS_INFO_STREAM("impedanceStartControl strating ! ");
    is_running=true;
    is_stop= false;
    //阻抗插件初始化
    cout<<"algorithm_name: "<<yamlPara_algorithm_name<<endl;
    if(force_plugin->setForcePlugin(yamlPara_algorithm_name, "1", "")!=0){
        return -1;
    }
    //设置阻抗算法参数
    force_plugin->setParameters(&yamlPara_Stiffness[0],&yamlPara_Damping[0],&yamlPara_Mass[0]);
    //循环前发布一次起始关节角
    vector<double> startPos = MG.move_group->getCurrentJointValues();
    MG.kinematic_state->setJointGroupPositions( MG.joint_model_group, startPos);
    if(isSim)
    {
        robot_servo_status=true;
        publishPose(startPos);
    }
    else
    {
        publishOnceForRealRb(startPos);//使用“impedance_err”接口驱动真机，第一次发送数据格式不同，必须要发。
    }
    while(ros::ok()&&(!is_stop)&&(!ros::isShuttingDown())&&(robot_servo_status))
    {
        auto start = boost::chrono::system_clock::now();

        vector<double > copy_currentForce=currentForce;
        vector<double > out_dealForce;
        forceDataDeal(copy_currentForce,out_dealForce);

        std::vector<double> outJoint;
        geometry_msgs::Pose outPose;
        if(computeImpedence(out_dealForce, outJoint,outPose)!=0){
            continue;
//            return -1;
        }
        //发布位姿
        Pose_state_pub.publish(outPose);
        //执行运动
        publishPose(outJoint);
        boost::this_thread::sleep_until( start +boost::chrono::milliseconds(PUBPOSE_HZ));
        ROS_INFO_STREAM("<---------------------------------------------------->");
    }
    is_running= false;
    return 0;
}

//停止阻抗控制
int forceService::StopImpedenceCtl() {
    is_stop=true;
    int time_count=0;
    while (is_running&&(time_count<10)){
        sleep(1);
        time_count++;
    }
    return 0;
}

//读取本地参数
int forceService::readLocalParam() {
    //获取当前包路径
    string package_path = ros::package::getPath("force_bridge");
    string configFile_path=package_path+"/config/forceImp.yaml";
    YAML::Node yaml_node=YAML::LoadFile(configFile_path);
    //参数解析
    yamlPara_forceScale=yaml_node["parameters"]["revForceScale"].as<vector<double >>();
    if (yamlPara_forceScale.size()!=3){
        return -1;
    }
    yamlPara_forceDrection=yaml_node["parameters"]["forceDrectionEnable"].as<vector<bool >>();
    if (yamlPara_forceScale.size()!=3){
        return -2;
    }
    safetyAreaScope.resize(3);
    safetyAreaScope[0]=yaml_node["parameters"]["Limit_safetyAreaScope_x"].as<vector<double >>();
    safetyAreaScope[1]=yaml_node["parameters"]["Limit_safetyAreaScope_y"].as<vector<double >>();
    safetyAreaScope[2]=yaml_node["parameters"]["Limit_safetyAreaScope_z"].as<vector<double >>();

    yamlPara_Stiffness=yaml_node["parameters"]["Stiffness"].as<vector<double >>();
    yamlPara_Damping=yaml_node["parameters"]["Damping"].as<vector<double >>();
    yamlPara_Mass=yaml_node["parameters"]["Mass"].as<vector<double >>();

    yamlPara_algorithm_name=yaml_node["parameters"]["algorithm_name"].as<string >();
//    cout<<"algorithm_name: "<<yamlPara_algorithm_name<<endl;

     yamlPara_MaxVel_x=yaml_node["parameters"]["MaxVel_x"].as<double >();
     yamlPara_MaxVel_y=yaml_node["parameters"]["MaxVel_y"].as<double >();
     yamlPara_MaxVel_z=yaml_node["parameters"]["MaxVel_z"].as<double >();
    return 0;
}

//更新阻抗参数
int forceService::updateParam() {
    return 0;
}

//设置阻抗算法的输入传感器来源
int forceService::setForceInputTopic() {
    return 0;
}

//设置阻抗控制的方向
int forceService::setForceControlDirect() {

    return 0;
}

bool forceService::initKinematic() {
    try{
        MG.move_group = new moveit::planning_interface::MoveGroupInterface(MG.groupName);
    }catch(std::runtime_error &e){
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    MG.move_group->setStartState(*MG.move_group->getCurrentState());

    vector<double> startPos = MG.move_group->getCurrentJointValues();
    assert(startPos.size() > 0);
    MG.kinematic_model = MG.move_group->getRobotModel();
    MG.kinematic_state = MG.move_group->getCurrentState();
    //kinematic_state->setToDefaultValues();
    MG.joint_model_group = const_cast<robot_state::JointModelGroup*>(MG.kinematic_model->getJointModelGroup(MG.groupName));
    MG.joint_names = MG.joint_model_group->getJointModelNames();

    if(MG.joint_model_group == nullptr)
        return false;
    for (int i = 0; i <MG.joint_names.size(); ++i) {
        ROS_INFO_STREAM(MG.joint_names[i]);
    }
    ROS_INFO_STREAM("initKinematic IS OK");
    return true;
}

bool forceService::impedenceStartCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if(StartImpedenceCtl()==0){
        res.success = true;
    } else{
        res.success = false;
    }
    return true;
}

bool forceService::impedenceCloseCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    is_stop=true;
    while (is_running){
        sleep(1);
    }
    res.success = true;
    return true;
}

void forceService::forceCallbackXZ(const geometry_msgs::Wrench::ConstPtr &msg) {
    currentForce[0] = msg->force.x * yamlPara_forceScale[0] ;
    currentForce[1] = -msg->force.y * yamlPara_forceScale[1];
    currentForce[2] = -msg->force.z * yamlPara_forceScale[2];
    currentForce[3] = 0;
    currentForce[4] = 0;
    currentForce[5] = 0;
}

void forceService::forceCallbackZX(const geometry_msgs::Wrench_<allocator<void>>::ConstPtr &msg) {
    currentForce[0] = msg->force.z * yamlPara_forceScale[0];
    currentForce[1] = -msg->force.y * yamlPara_forceScale[1];
    currentForce[2] = msg->force.x * yamlPara_forceScale[2];
    currentForce[3] = 0;
    currentForce[4] = 0;
    currentForce[5] = 0;

}


void forceService::robotStausCallback(const industrial_msgs::RobotStatusConstPtr &msg) {
//    if(msg->in_error.val != 0 || msg->in_motion.val != 0 ){
    if(msg->in_error.val != 0 || msg->drives_powered.val!=1 ){
        robot_servo_status = false;
        std::cout << "msg->in_motion.val : "<< std::to_string(msg->in_motion.val) << " msg->in_error.val: "<<  std::to_string(msg->in_error.val)<<std::endl;
        robot_status_sub.shutdown();
    }else {
        robot_servo_status = true;
    }
}

void forceService::publishOnceForRealRb(std::vector<double> &startPos) {
    startPos.push_back(0);
    sensor_msgs::JointState sensor_compute_robot_state;
    sensor_compute_robot_state.header.stamp = ros::Time::now();
    sensor_compute_robot_state.name.resize(7);
    sensor_compute_robot_state.position = startPos;
    joint_state_pub.publish(sensor_compute_robot_state);
    ROS_INFO_STREAM( "sensor_compute_robot_state size: " << sensor_compute_robot_state.position.size() );
    matrixUtily::dumpDVec(sensor_compute_robot_state.position, 7,"sensor_compute_robot_state: ");
    startPos.pop_back();

}

void forceService::publishPose(std::vector<double> &joint_Pose) {
    sensor_msgs::JointState compute_robot_state;
    compute_robot_state.header.stamp = ros::Time::now();
    compute_robot_state.name.resize(6);
    for (int i = 0; i <6; ++i) {
        compute_robot_state.name[i]=MG.joint_names[i+1];
    }
    compute_robot_state.position = joint_Pose;
    //ROS_INFO_STREAM(compute_robot_state);
    //运动点打印
    vector<double> tmp(6);
    for (size_t i = 0; i < 6; i++)
    {
        tmp[i]=compute_robot_state.position[i]*180/M_PI;
        cout<<"点位执行关节"<<i<<"角度值: "<<compute_robot_state.position[i]*180/M_PI<<endl;;
    }
    // if(tmp[1]<=-72){
    //     cout<<"轴2非安全点"<<endl;
    //     compute_robot_state.position[1]=(-72.0/180.0)*M_PI;
    // }
    // if(tmp[2]>=210){
    //     cout<<"轴3非安全点"<<endl;
    //     compute_robot_state.position[2]=(210.0/180.0)*M_PI;
    // }
    // if(tmp[4]<=-60){
    //     cout<<"轴5非安全点"<<endl;
    //     compute_robot_state.position[4]=(-60.0/180.0)*M_PI;
    // }

    joint_state_pub.publish(compute_robot_state);
    

}

void forceService::getCurrentPose(geometry_msgs::Pose& Pose) {
    // 获取当前的末端姿态
//    cout<<"w1"<<endl;
//    vector<double> curPos = MG.move_group->getCurrentJointValues();
//    cout<<"w2"<<endl;
//    MG.kinematic_state->setJointGroupPositions( MG.joint_model_group, curPos);
    const Eigen::Affine3d &end_effector_state =  MG.kinematic_state->getGlobalLinkTransform( MG.endlinkName);
    tf::poseEigenToMsg(end_effector_state, Pose);
}

int forceService::computeImpedence(std::vector<double> &force, std::vector<double> &outJoint,geometry_msgs::Pose &outPose) {
    assert(force.size() == 6);
    //1.获取当前位姿
    geometry_msgs::Pose current_Pose;
    getCurrentPose(current_Pose);
    outPose=current_Pose;
    //2.阻抗运算
    std::vector<double> Xa(6);
    std::fill(Xa.begin(), Xa.begin()+6, 0);
    vector<double > tmp_pose{0,0,0,0,0,0};
    force_plugin->setInputRobotPose(tmp_pose);

    force_plugin->setInputForceBias(force);
    int i = force_plugin->compute();
    int result = force_plugin->getResult(Xa);
    //位移量控制
    if(Xa[0]>=yamlPara_MaxVel_x){
        Xa[0]=yamlPara_MaxVel_x;
    }else if(Xa[0]<= -1*yamlPara_MaxVel_x){
        Xa[0]=-1*yamlPara_MaxVel_x;
    }

    if(Xa[1]>=yamlPara_MaxVel_y){
        Xa[1]=yamlPara_MaxVel_y;
    }else if(Xa[1]<= -1*yamlPara_MaxVel_y){
        Xa[1]=-1*yamlPara_MaxVel_y;
    }

    if(Xa[2]>=yamlPara_MaxVel_z){
        Xa[2]=yamlPara_MaxVel_z;
    }else if(Xa[2]<= -1*yamlPara_MaxVel_z){
        Xa[2]=-1*yamlPara_MaxVel_z;
    }

    cout<<"计算得偏移量X_offset: "<<Xa[0]<<endl;
    cout<<"计算得偏移量y_offset: "<<Xa[1]<<endl;
    cout<<"计算得偏移量z_offset: "<<Xa[2]<<endl;

    double diff=0;
    std::vector<double> joint_values;
    vector<double > curJoint;
    geometry_msgs::Pose computePose;

    //3.位姿补偿计算
    computePose = current_Pose;
    computePose.position.x+=Xa[0];
    computePose.position.y+=Xa[1];
    computePose.position.z+=Xa[2];

    //4.位姿转关节角
    if(!MG.kinematic_state->setFromIK(MG.joint_model_group, computePose, MG.endlinkName, 10, 0.1)){
        ROS_ERROR( "IK ERR " );
        return -1;
    }

    // 返回计算后的关节角
    MG.kinematic_state->copyJointGroupPositions(MG.joint_model_group, joint_values);
    //关节角偏移量
    // curJoint =MG.move_group->getCurrentJointValues();
    // for (size_t i = 0; i < 6; i++)
    // {
    //     diff=(joint_values[i]-curJoint[i])*180/M_PI;
    //     cout<<"计算偏移量joint"<<i<<"偏差角度值: "<<setprecision(2)<<diff<<endl;
    //     if((diff<-0.6)||(diff>0.6))
    //     {
    //         flag=true;
    //     }
    // }

    outJoint =  std::move(joint_values);
    return 0;
}

void forceService::forceDataDeal(const vector<double >& original_force,vector<double >& deal_force) {
    assert(original_force.size() == 6);
    vector<double > tmp_deal_force(6);
    std::fill(tmp_deal_force.begin(),tmp_deal_force.begin()+6,0);

    if(!flag_SetForceBias){
        bias_force=currentForce;
        flag_SetForceBias=true;
    }

    //只管X,Y,Z三个方向力矩
    for (int i = 0; i <3; ++i) {
        if(yamlPara_forceDrection[i]){
            tmp_deal_force[i]=original_force[i]-bias_force[i];
        }
    }
    deal_force=tmp_deal_force;
}

bool forceService::force_algorithmChangeCB(hirop_msgs::force_algorithmChange::Request &req,
                                           hirop_msgs::force_algorithmChange::Response &res) {
    yamlPara_algorithm_name=req.algorithm_name;
    res.is_success=true;
    return true;
}

