#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"
#include "DobotDll.h"
#include <iostream>

#include "dobot/SetWAITCmd.h"
#include "dobot/GetPose.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"
//#include "dobot/SetEMotor.h"
#include "dobot/SetEndEffectorSuctionCup.h"
#include "dobot/SetColorSensor.h"
#include "dobot/GetColorSensor.h"
#include "dobot/SetInfraredSensor.h"
#include "dobot/GetInfraredSensor.h"

#include "MainAgent.h"
using namespace std;

Pose cur_pose_1;

MainAgent::MainAgent(){
	ros::NodeHandle pn("~");
	//Initialize();
}

MainAgent::~MainAgent(){
}

int MainAgent::Initialize(ros::NodeHandle &nh)
{
	//ros::NodeHandle n;
	//ros::NodeHandle nh("~");

    ros::ServiceClient mainInit;

    // SetCmdTimeout
    mainInit = nh.serviceClient<dobot::SetCmdTimeout>("/DobotAgent/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (mainInit.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    // Clear the command queue
    mainInit = nh.serviceClient<dobot::SetQueuedCmdClear>("/DobotAgent/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    mainInit.call(srv2);

    // Start running the command queue
    mainInit = nh.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotAgent/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    mainInit.call(srv3);

    // Get device version information
    mainInit = nh.serviceClient<dobot::GetDeviceVersion>("/DobotAgent/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    mainInit.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // Set end effector parameters
    mainInit = nh.serviceClient<dobot::SetEndEffectorParams>("/DobotAgent/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    mainInit.call(srv5);

    // Set PTP joint parameters
    do {
        mainInit = nh.serviceClient<dobot::SetPTPJointParams>("/DobotAgent/SetPTPJointParams");
        dobot::SetPTPJointParams srv;

        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        mainInit.call(srv);
    } while (0);

    // Set PTP coordinate parameters
    do {
        mainInit = nh.serviceClient<dobot::SetPTPCoordinateParams>("/DobotAgent/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        mainInit.call(srv);
    } while (0);

    // Set PTP jump parameters
    do {
        mainInit = nh.serviceClient<dobot::SetPTPJumpParams>("/DobotAgent/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        mainInit.call(srv);
    } while (0);

    // Set PTP common parameters
    do {
        mainInit = nh.serviceClient<dobot::SetPTPCommonParams>("/DobotAgent/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv;
 
        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        mainInit.call(srv);
    } while (0);


    // Set end effector parameters
    mainInit = nh.serviceClient<dobot::SetEndEffectorParams>("/DobotAgent/SetEndEffectorParams");
    dobot::SetEndEffectorParams srvep;
    srvep.request.xBias = 59.7;
    srvep.request.yBias = 0;
    srvep.request.zBias = 0;
    srvep.request.isQueued = 1;
    mainInit.call(srvep);

    // Set color sensor
    mainInit = nh.serviceClient<dobot::SetColorSensor>("/DobotAgent/SetColorSensor");
    dobot::SetColorSensor srvcs;
    srvcs.request.colorPort = 2; //GP4
    srvcs.request.enable = 1;
    mainInit.call(srvcs);

    // Set infrared sensor
    mainInit = nh.serviceClient<dobot::SetInfraredSensor>("/DobotAgent/SetInfraredSensor");
    dobot::SetInfraredSensor srvis;
    srvis.request.infraredPort = 1; //GP2
    srvis.request.enable = 1;
    mainInit.call(srvis);

    // Get current position
    mainInit = nh.serviceClient<dobot::GetPose>("/DobotAgent/GetPose");
    dobot::GetPose srvcp;    
    mainInit.call(srvcp);
    cur_pose_1.x = srvcp.response.x;
    cur_pose_1.y = srvcp.response.y;
    cur_pose_1.z = srvcp.response.z;
    cur_pose_1.r = srvcp.response.r;

    // Delay
    mainInit = nh.serviceClient<dobot::SetWAITCmd>("/DobotAgent/SetWAITCmd");
    dobot::SetWAITCmd srvw;
    srvw.request.timeout = 1;
    //srvw.request.isQueued = 1;
    mainInit.call(srvw);

    // PTP mode jump to the initial location
    mainInit = nh.serviceClient<dobot::SetPTPCmd>("/DobotAgent/SetPTPCmd");
    dobot::SetPTPCmd srvptp; 
    srvptp.request.ptpMode = 0;
    srvptp.request.x = INIT_X; // PTP mode = 0
    srvptp.request.y = INIT_Y;
    srvptp.request.z = INIT_Z;
    srvptp.request.r = cur_pose_1.r;
    mainInit.call(srvptp);

    // Set the suctioncup to off
    mainInit = nh.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotAgent/SetEndEffectorSuctionCup");
    dobot::SetEndEffectorSuctionCup srvs;
    srvs.request.enableCtrl = 1;
    srvs.request.suck = 0;
    srvs.request.isQueued = 1;       
    mainInit.call(srvs);
}