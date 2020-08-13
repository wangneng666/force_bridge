#include "forceService.h"
forceService::forceService(ros::NodeHandle n) {
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
    Node.getParam("/force_bridge/group_name",MG.groupName);
    Node.getParam("/force_bridge/endlinkName",MG.endlinkName);
    Node.getParam("/force_bridge/isSim",isSim);
    Node.getParam("/force_bridge/XZReverseDirect",XZReverseDirect);
    ROS_INFO_STREAM("groupName "<<MG.groupName);
    ROS_INFO_STREAM("endlinkName "<<MG.endlinkName);
    ROS_INFO_STREAM("isSim "<<isSim);
    ROS_INFO_STREAM("XZReverseDirect: "<<XZReverseDirect);
    //阻抗插件初始化
    int ret = force_plugin->setForcePlugin("hsImpenderrForce", "1", "");
    cout<<force_plugin->getName()<<endl;

    //变量初始化
    std::fill(bias_force.begin(), bias_force.begin()+6 ,0);
    //话题初始化
    if(isSim)
        joint_state_pub = Node.advertise<sensor_msgs::JointState>("joint_states", 1);
    else
        joint_state_pub = Node.advertise<sensor_msgs::JointState>("impedance_err", 1);

    //根据随动方西选择数据
    if(!XZReverseDirect)
        force_sub = Node.subscribe("/daq_data", 1, &forceService::forceCallbackXZ, this);
    else
        force_sub = Node.subscribe("/daq_data", 1, &forceService::forceCallbackZX, this);

    impedenceStart_server = Node.advertiseService("impedance_control", &forceService::impedanceStartCB,this);

    return ret;

}

//开启阻抗控制
int forceService::StartImpedenceCtl() {
    ROS_INFO_STREAM("impedanceStartControl strating ! ");
    //循环前发布一次起始关节角
    vector<double> startPos = MG.move_group->getCurrentJointValues();
    publishPose(startPos);
    while(ros::ok())
    {
        auto start = boost::chrono::system_clock::now();

        std::vector<double> currentForce(this->currentForce);
        std::vector<double> currPose;
        computeImpedance( currentForce, outJoint, currPose);
        publishPose(outJoint);
        ROS_INFO_STREAM("<---------------------------------------------------->");
        boost::this_thread::sleep_until( start +boost::chrono::milliseconds(40));
    }

    return 0;
}

//停止阻抗控制
int forceService::StopImpedenceCtl() {

    return 0;
}

//读取本地参数
int forceService::readLocalParam() {
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

bool forceService::impedanceStartCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    ROS_INFO_STREAM("impedanceStartControl strating ! ");
    bool status = prepareImpedance();
    if(!status){
        res.success = -1;
        return true;
    }
    ROS_INFO_STREAM("into blockLoop ! ");
    int ret = blockLoop();
    res.success = ret;
    return true;
}

bool forceService::prepareImpedance() {
    //1.初始化起始关节坐标
//    MG.move_group->setStartStateToCurrentState();
    vector<double> tempPos = MG.move_group->getCurrentJointValues();
    startPos = vector<double>(6);
    copy(tempPos.begin(), tempPos.begin()+6, startPos.begin());
    //2.发布起始关节坐标  (是否可以优化掉？)
    publishStartPose(startPos);
    //3.获取当前位姿　　　(是否可以优化掉？)
    initRobotMoveitPose(startPos);
    /********************/
    //4.初始化传感器偏差值
    std::fill(bias_force.begin(), bias_force.begin()+6 ,0);
    getForce = false;
    ready_exit = false;
    while(getForce == false)
    {
        if(ready_exit)
            return false;
    }
    vector<double> biasFValue(currentForce);
    for(int i = 0; i< 6; i++)
        bias_force[i] = biasFValue[i];

    //等待初始状态与机器人状态获取成功
//    robotStartStatus = false;
//    ROS_INFO_STREAM("ready the robot status ok ...");
//    robot_status_sub = Node.subscribe("/robot_status", 1, &forceService::robotStausCallback, this);
//
//
//    //等待机器人状态成功
//    while(robot_servo_status == false){
//        if(ready_exit)
//            return false;
//    }
    ROS_INFO_STREAM("prepareImpedance is end !");

    return true;
}

int forceService::blockLoop() {
    //握手的点位下发
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(6);
    for(int i = 0; i < 6; i++){
        joint_state.name[i] = MG.joint_names[i+1];
    }
    ROS_INFO_STREAM(joint_state);

    //握手的输出结果
    std_msgs::Int16 ret;
    // 输出力矩的关节位置
    std::vector<double> outJoint(6);
    ret.data = -3;

//    std::vector<double> currentForce(this->currentForce);
//    std::vector<double> currPose;
//    bool status = computeImpedance( currentForce, outJoint, currPose);
    system("rosnode kill /joint_state_publisher &");
    sleep(1);
    while(ros::ok())
    {
        auto start = boost::chrono::system_clock::now();
//        if(!robot_servo_status){
//            ret.data = -1;
//            break;
//        }
        // block the force
        std::vector<double> currentForce(this->currentForce);
        std::vector<double> currPose;
        bool status = computeImpedance( currentForce, outJoint, currPose);
        if(!status){
            ret.data = -2;
            break;
        }
//        outJoint[0]+=0.001;
        //publish
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position = outJoint;
        ROS_INFO_STREAM(joint_state);
        joint_state_pub.publish(joint_state);
//        system("rosrun joint_state_publisher joint_state_publisher &");
        ROS_INFO_STREAM("<---------------------------------------------------->");
        boost::this_thread::sleep_until( start +boost::chrono::milliseconds(40));
    }
    impenderrResult.publish(ret);
    ROS_ERROR_STREAM("robot_servo_status error exit! "<< ret.data);
    robot_status_sub.shutdown();
    return ret.data;
}

void forceService::publishStartPose(std::vector<double> &robotStartPose) {
//    startPos.push_back(0);
    sensor_msgs::JointState sensor_compute_robot_state;
    sensor_compute_robot_state.header.stamp = ros::Time::now();
//    sensor_compute_robot_state.name.resize(7);
    sensor_compute_robot_state.name.resize(6);
    for (int i = 0; i <6; ++i) {
        sensor_compute_robot_state.name[i]=MG.joint_names[i+1];
    }
    sensor_compute_robot_state.position = startPos;
    ROS_INFO_STREAM(sensor_compute_robot_state);
    joint_state_pub.publish(sensor_compute_robot_state);
    ROS_INFO_STREAM( "sensor_compute_robot_state size: " << sensor_compute_robot_state.position.size() );
//    matrixUtily::dumpDVec(sensor_compute_robot_state.position, 7,"sensor_compute_robot_state: ");
//    startPos.pop_back();
}


void forceService::initRobotMoveitPose(const std::vector<double> &robotStartPose) {
    // 获取当前的末端姿态
    MG.kinematic_state->setJointGroupPositions( MG.joint_model_group, robotStartPose);
    const Eigen::Affine3d &end_effector_state =  MG.kinematic_state->getGlobalLinkTransform( MG.endlinkName);
    tf::poseEigenToMsg(end_effector_state, SPose);
    // 四元数转RPY
    matrixUtily::QtoETool(SPose.orientation.x, SPose.orientation.y, SPose.orientation.z, SPose.orientation.w, SA, SB, SC);
    matrixUtily::dumpDVec(robotStartPose, 6,"robotStartPose joint pose: ");

}

void forceService::forceCallbackXZ(const geometry_msgs::Wrench::ConstPtr &msg) {
    currentForce[0] = msg->force.x * 0.08 - bias_force[0];
    currentForce[1] = -msg->force.y * 0.08 - bias_force[1];
    currentForce[2] = -msg->force.z * 0.088 - bias_force[2]; // 605 luo
    currentForce[3] = 0;
    currentForce[4] = 0;
    currentForce[5] = 0;
    if(getForce== false){
        getForce = true;
        bias_force=currentForce;
    }
}

void forceService::forceCallbackZX(const geometry_msgs::Wrench_<allocator<void>>::ConstPtr &msg) {
    currentForce[0] = msg->force.z * 0.08 - bias_force[0];
    currentForce[1] = -msg->force.y * 0.08 - bias_force[1];
    currentForce[2] = msg->force.x * 0.082 - bias_force[2]; // 605 luo
    currentForce[3] = 0;
    currentForce[4] = 0;
    currentForce[5] = 0;
    if(getForce== false){
        getForce = true;
        bias_force=currentForce;
    }
}

//void forceService::initRosTopic() {
//    // 发布关节角数据
//    if(isSim)
//        joint_state_pub = Node.advertise<sensor_msgs::JointState>("joint_states", 1);
//    else
//        joint_state_pub = Node.advertise<sensor_msgs::JointState>("impedance_err", 1);
//
//    impenderrResult = Node.advertise<std_msgs::Int16>("impedance_result",1);
////    ready_stop_sub = Node.subscribe<std_msgs::Bool>("set_ready_exit", 1, &forceService::set_ready_callback,this);
//    // 订阅机械臂起始关节角
//    //start_pos_sub = Node.subscribe("/joint_states", 1, &impedaceServer::startPosCallback ,this);
//
//    // 订阅传感器数据
//    XZReverseDirect = false;
//    currentForce = vector<double>(6);
//
//    std::fill(currentForce.begin(), currentForce.end(), 0);
//    bias_force.resize(6);
//    std::copy(currentForce.begin(), currentForce.end(), bias_force.begin());
//    if(!XZReverseDirect)
//        force_sub = Node.subscribe("/daq_data", 1, &forceService::forceCallbackXZ, this);
//    else
//        force_sub = Node.subscribe("/daq_data", 1, &forceService::forceCallbackZX, this);
//}

//void forceService::initRosServer(){
//    impedenceStart_server = Node.advertiseService("impedance_control", &forceService::impedanceStartCB,this);
//}

bool forceService::computeImpedance(std::vector<double> &force, std::vector<double> &outJoint,std::vector<double> &outPos) {
    double A_offset = 0, B_offset = 0, C_offset = 0, X_offset = 0,  Y_offset = 0,  Z_offset = 0;
    double A = SA, B = SB , C = SC;
    //计算当前末端位姿
//    MG.kinematic_state->setJointGroupPositions(MG.joint_model_group, startPos);
    const Eigen::Affine3d &end_effector_state = MG.kinematic_state->getGlobalLinkTransform(MG.endlinkName);
    tf::poseEigenToMsg(end_effector_state, SPose);

//    geometry_msgs::PoseStamped stamped = MG.move_group->getCurrentPose("link6");
//    system("rosnode kill /joint_state_publisher &");
//    SPose.position=stamped.pose.position;
//    SPose.orientation=stamped.pose.orientation;
    geometry_msgs::Pose computePose = SPose;
    ROS_INFO_STREAM("SPOSE= "<<SPose);
    std::vector<double> Xa(6);
    std::fill(Xa.begin(), Xa.begin()+6, 0);

    // 计算偏移值
//    vector<double > tmp_pose= MG.move_group->getCurrentJointValues();
    vector<double > tmp_pose{0,0,0,0,0,0};
    force_plugin->setInputRobotPose(tmp_pose);
    force_plugin->setInputForceBias(force);
    int i = force_plugin->compute();
    int result = force_plugin->getResult(Xa);

    X_offset = Xa[0];
    Y_offset = Xa[1];
    Z_offset = Xa[2];
//    X_offset = 0.0;
//    Y_offset = 0.0;
//    Z_offset = 0.01;
    A_offset = Xa[3];B_offset = Xa[4];C_offset = Xa[5];
    cout<<"计算得偏移量X_offset: "<<X_offset<<endl;
    cout<<"计算得偏移量y_offset: "<<Y_offset<<endl;
    cout<<"计算得偏移量z_offset: "<<Z_offset<<endl;
    // 偏移 calucation
    A += A_offset; B += B_offset; C += C_offset;
    computePose.position.x += X_offset; computePose.position.y += Y_offset; computePose.position.z += Z_offset;
    ROS_INFO_STREAM("compute effector cart pose XYZ ABC: "<< computePose.position.x<<" "<< computePose.position.y<<" "<< computePose.position.z   <<" "
                                                          <<A<<" "<< B<<" "<<C);
    // 偏移后RPY转四元数
    tf::Quaternion q;
    q.setEulerZYX(C, B, A);
    tf::Vector3 axis = q.getAxis();
    computePose.orientation.x = axis.getX();computePose.orientation.y = axis.getY();
    computePose.orientation.z = axis.getZ();computePose.orientation.w = q.getW();

    //inverse
    if(!MG.kinematic_state->setFromIK(MG.joint_model_group, computePose, MG.endlinkName, 10, 0.1)){
        ROS_ERROR( "运动学逆解失败 " );
        matrixUtily::dumpDVec(outJoint, 6, " outPose ");
        return false;
    }

    // 返回计算后的关节角
    std::vector<double> joint_values;
    MG.kinematic_state->copyJointGroupPositions(MG.joint_model_group, joint_values);

    outJoint =  std::move(joint_values);
    outPos =  std::vector<double>{SPose.position.x,SPose.position.y,SPose.position.z};

    return true;
}

void forceService::robotStausCallback(const industrial_msgs::RobotStatusConstPtr &msg) {
//    if(msg->in_error.val != 0 || msg->in_motion.val != 0 ){
    if(msg->in_error.val != 0 || msg->drives_powered.val!=1 ){
        robot_servo_status = false;
        std::cout << "msg->in_motion.val : "<< std::to_string(msg->in_motion.val) << " msg->in_error.val: "<<  std::to_string(msg->in_error.val)<<std::endl;
    }else if( msg->in_error.val  == 0 && robotStartStatus == false ){
        robot_servo_status = true;
        robotStartStatus = true;
    }
}

void forceService::publishPose(std::vector<double> &joint_Pose) {
    sensor_msgs::JointState compute_robot_state;
    compute_robot_state.header.stamp = ros::Time::now();
    compute_robot_state.name.resize(6);
    for (int i = 0; i <6; ++i) {
        compute_robot_state.name[i]=MG.joint_names[i+1];
    }
    compute_robot_state.position = joint_Pose;
//    ROS_INFO_STREAM(compute_robot_state);
    joint_state_pub.publish(compute_robot_state);


}

void forceService::getCurrentPose(geometry_msgs::Pose& Pose) {
    // 获取当前的末端姿态
    vector<double> curPos = MG.move_group->getCurrentJointValues();
    MG.kinematic_state->setJointGroupPositions( MG.joint_model_group, curPos);
    const Eigen::Affine3d &end_effector_state =  MG.kinematic_state->getGlobalLinkTransform( MG.endlinkName);
    tf::poseEigenToMsg(end_effector_state, Pose);
}

