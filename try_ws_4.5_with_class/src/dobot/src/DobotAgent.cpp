#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include "std_msgs/Float32MultiArray.h"
#include "DobotDll.h"
#include "DobotAgent.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace std;

DobotAgent::DobotAgent(){
	//ros::NodeHandle pn("~");
}

DobotAgent::~DobotAgent(){
}

/*
 * Cmd timeout
 */
#include "dobot/SetCmdTimeout.h"

bool DobotAgent::SetCmdTimeoutService(dobot::SetCmdTimeout::Request &req, dobot::SetCmdTimeout::Response &res)
{
    res.result = SetCmdTimeout(req.timeout);

    return true;
}

void DobotAgent::InitCmdTimeoutServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetCmdTimeout", &DobotAgent::SetCmdTimeoutService, this);
    serverVec.push_back(server);
}

/*
 * Device information
 */
#include "dobot/GetDeviceSN.h"
#include "dobot/SetDeviceName.h"
#include "dobot/GetDeviceName.h"
#include "dobot/GetDeviceVersion.h"

bool DobotAgent::GetDeviceSNService(dobot::GetDeviceSN::Request &req, dobot::GetDeviceSN::Response &res)
{
    char deviceSN[256];

    res.result = GetDeviceSN(deviceSN, sizeof(deviceSN));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceSN;
        res.deviceSN.data = ss.str();
    }

    return true;
}

bool DobotAgent::SetDeviceNameService(dobot::SetDeviceName::Request &req, dobot::SetDeviceName::Response &res)
{
    res.result = SetDeviceName(req.deviceName.data.c_str());

    return true;
}

bool DobotAgent::GetDeviceNameService(dobot::GetDeviceName::Request &req, dobot::GetDeviceName::Response &res)
{
    char deviceName[256];

    res.result = GetDeviceName(deviceName, sizeof(deviceName));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceName;
        res.deviceName.data = ss.str();
    }

    return true;
}

bool DobotAgent::GetDeviceVersionService(dobot::GetDeviceVersion::Request &req, dobot::GetDeviceVersion::Response &res)
{
    uint8_t majorVersion, minorVersion, revision;

    res.result = GetDeviceVersion(&majorVersion, &minorVersion, &revision);
    if (res.result == DobotCommunicate_NoError) {
        res.majorVersion = majorVersion;
        res.minorVersion = minorVersion;
        res.revision = revision;
    }

    return true;
}

void DobotAgent::InitDeviceInfoServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/GetDeviceSN", &DobotAgent::GetDeviceSNService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetDeviceName", &DobotAgent::SetDeviceNameService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetDeviceName", &DobotAgent::GetDeviceNameService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetDeviceVersion", &DobotAgent::GetDeviceVersionService, this);
    serverVec.push_back(server);
}

/*
 * Pose
 */
#include "dobot/GetPose.h"

bool DobotAgent::GetPoseService(dobot::GetPose::Request &req, dobot::GetPose::Response &res)
{
    Pose pose;

    res.result = GetPose(&pose);
    if (res.result == DobotCommunicate_NoError) {
        res.x = pose.x;
        res.y = pose.y;
        res.z = pose.z;
        res.r = pose.r;
        for (int i = 0; i < 4; i++) {
            res.jointAngle.push_back(pose.jointAngle[i]);
        }
    }

    return true;
}

void DobotAgent::InitPoseServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/GetPose", &DobotAgent::GetPoseService, this);
    serverVec.push_back(server);
}

/*
 * Alarms
 */
#include "dobot/GetAlarmsState.h"
#include "dobot/ClearAllAlarmsState.h"

bool DobotAgent::GetAlarmsStateService(dobot::GetAlarmsState::Request &req, dobot::GetAlarmsState::Response &res)
{
    uint8_t alarmsState[128];
    uint32_t len;

    res.result = GetAlarmsState(alarmsState, &len, sizeof(alarmsState));
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < len; i++) {
            res.alarmsState.push_back(alarmsState[i]);
        }
    }

    return true;
}

bool DobotAgent::ClearAllAlarmsStateService(dobot::ClearAllAlarmsState::Request &req, dobot::ClearAllAlarmsState::Response &res)
{
    res.result = ClearAllAlarmsState();

    return true;
}

void DobotAgent::InitAlarmsServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/GetAlarmsState", &DobotAgent::GetAlarmsStateService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/ClearAllAlarmsState", &DobotAgent::ClearAllAlarmsStateService, this);
    serverVec.push_back(server);
}

/*
 * HOME
 */
#include "dobot/SetHOMEParams.h"
#include "dobot/GetHOMEParams.h"
#include "dobot/SetHOMECmd.h"

bool DobotAgent::SetHOMEParamsService(dobot::SetHOMEParams::Request &req, dobot::SetHOMEParams::Response &res)
{
    HOMEParams params;
    uint64_t queuedCmdIndex;

    params.x = req.x;
    params.y = req.y;
    params.z = req.z;
    params.r = req.r;

    res.result = SetHOMEParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetHOMEParamsService(dobot::GetHOMEParams::Request &req, dobot::GetHOMEParams::Response &res)
{
    HOMEParams params;

    res.result = GetHOMEParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.x = params.x;
        res.y = params.y;
        res.z = params.z;
        res.r = params.r;
    }

    return true;
}

bool DobotAgent::SetHOMECmdService(dobot::SetHOMECmd::Request &req, dobot::SetHOMECmd::Response &res)
{
    HOMECmd cmd;
    uint64_t queuedCmdIndex;

    res.result = SetHOMECmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void DobotAgent::InitHOMEServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetHOMEParams", &DobotAgent::SetHOMEParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetHOMEParams", &DobotAgent::GetHOMEParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetHOMECmd", &DobotAgent::SetHOMECmdService, this);
    serverVec.push_back(server);
}

/*
 * End effector
 */
#include "dobot/SetEndEffectorParams.h"
#include "dobot/GetEndEffectorParams.h"
#include "dobot/SetEndEffectorLaser.h"
#include "dobot/GetEndEffectorLaser.h"
#include "dobot/SetEndEffectorSuctionCup.h"
#include "dobot/GetEndEffectorSuctionCup.h"
#include "dobot/SetEndEffectorGripper.h"
#include "dobot/GetEndEffectorGripper.h"

bool DobotAgent::SetEndEffectorParamsService(dobot::SetEndEffectorParams::Request &req, dobot::SetEndEffectorParams::Response &res)
{
    EndEffectorParams params;
    uint64_t queuedCmdIndex;

    params.xBias = req.xBias;
    params.yBias = req.yBias;
    params.zBias = req.zBias;

    res.result = SetEndEffectorParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetEndEffectorParamsService(dobot::GetEndEffectorParams::Request &req, dobot::GetEndEffectorParams::Response &res)
{
    EndEffectorParams params;

    res.result = GetEndEffectorParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xBias = params.xBias;
        res.yBias = params.yBias;
        res.zBias = params.zBias;
    }

    return true;
}

bool DobotAgent::SetEndEffectorLaserService(dobot::SetEndEffectorLaser::Request &req, dobot::SetEndEffectorLaser::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorLaser(req.enableCtrl, req.on, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetEndEffectorLaserService(dobot::GetEndEffectorLaser::Request &req, dobot::GetEndEffectorLaser::Response &res)
{
    bool enableCtrl, on;

    res.result = GetEndEffectorLaser(&enableCtrl, &on);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.on = on;
    }

    return true;
}

bool DobotAgent::SetEndEffectorSuctionCupService(dobot::SetEndEffectorSuctionCup::Request &req, dobot::SetEndEffectorSuctionCup::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorSuctionCup(req.enableCtrl, req.suck, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetEndEffectorSuctionCupService(dobot::GetEndEffectorSuctionCup::Request &req, dobot::GetEndEffectorSuctionCup::Response &res)
{
    bool enableCtrl, suck;

    res.result = GetEndEffectorLaser(&enableCtrl, &suck);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.suck = suck;
    }

    return true;
}

bool DobotAgent::SetEndEffectorGripperService(dobot::SetEndEffectorGripper::Request &req, dobot::SetEndEffectorGripper::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorGripper(req.enableCtrl, req.grip, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetEndEffectorGripperService(dobot::GetEndEffectorGripper::Request &req, dobot::GetEndEffectorGripper::Response &res)
{
    bool enableCtrl, grip;

    res.result = GetEndEffectorLaser(&enableCtrl, &grip);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.grip = grip;
    }

    return true;
}

void DobotAgent::InitEndEffectorServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetEndEffectorParams", &DobotAgent::SetEndEffectorParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetEndEffectorParams", &DobotAgent::GetEndEffectorParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetEndEffectorLaser", &DobotAgent::SetEndEffectorLaserService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetEndEffectorLaser", &DobotAgent::GetEndEffectorLaserService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetEndEffectorSuctionCup", &DobotAgent::SetEndEffectorSuctionCupService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetEndEffectorSuctionCup", &DobotAgent::GetEndEffectorSuctionCupService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetEndEffectorGripper", &DobotAgent::SetEndEffectorGripperService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetEndEffectorGripper", &DobotAgent::GetEndEffectorGripperService, this);
    serverVec.push_back(server);
}

/*
 * JOG
 */
#include "dobot/SetJOGJointParams.h"
#include "dobot/GetJOGJointParams.h"
#include "dobot/SetJOGCoordinateParams.h"
#include "dobot/GetJOGCoordinateParams.h"
#include "dobot/SetJOGCommonParams.h"
#include "dobot/GetJOGCommonParams.h"
#include "dobot/SetJOGCmd.h"

bool DobotAgent::SetJOGJointParamsService(dobot::SetJOGJointParams::Request &req, dobot::SetJOGJointParams::Response &res)
{
    JOGJointParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGJointParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetJOGJointParamsService(dobot::GetJOGJointParams::Request &req, dobot::GetJOGJointParams::Response &res)
{
    JOGJointParams params;

    res.result = GetJOGJointParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool DobotAgent::SetJOGCoordinateParamsService(dobot::SetJOGCoordinateParams::Request &req, dobot::SetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGCoordinateParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetJOGCoordinateParamsService(dobot::GetJOGCoordinateParams::Request &req, dobot::GetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;

    res.result = GetJOGCoordinateParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool DobotAgent::SetJOGCommonParamsService(dobot::SetJOGCommonParams::Request &req, dobot::SetJOGCommonParams::Response &res)
{
    JOGCommonParams params;
    uint64_t queuedCmdIndex;

    params.velocityRatio = req.velocityRatio;
    params.accelerationRatio = req.accelerationRatio;
    res.result = SetJOGCommonParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetJOGCommonParamsService(dobot::GetJOGCommonParams::Request &req, dobot::GetJOGCommonParams::Response &res)
{
    JOGCommonParams params;

    res.result = GetJOGCommonParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.velocityRatio = params.velocityRatio;
        res.accelerationRatio = params.accelerationRatio;
    }

    return true;
}

bool DobotAgent::SetJOGCmdService(dobot::SetJOGCmd::Request &req, dobot::SetJOGCmd::Response &res)
{
    JOGCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.isJoint = req.isJoint;
    cmd.cmd = req.cmd;
    res.result = SetJOGCmd(&cmd, false, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void DobotAgent::InitJOGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetJOGJointParams", &DobotAgent::SetJOGJointParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetJOGJointParams", &DobotAgent::GetJOGJointParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetJOGCoordinateParams", &DobotAgent::SetJOGCoordinateParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetJOGCoordinateParams", &DobotAgent::GetJOGCoordinateParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetJOGCommonParams", &DobotAgent::SetJOGCommonParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetJOGCommonParams", &DobotAgent::GetJOGCommonParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetJOGCmd", &DobotAgent::SetJOGCmdService, this);
    serverVec.push_back(server);
}

/*
 * PTP
 */
#include "dobot/SetPTPJointParams.h"
#include "dobot/GetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/GetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/GetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/GetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"

bool DobotAgent::SetPTPJointParamsService(dobot::SetPTPJointParams::Request &req, dobot::SetPTPJointParams::Response &res)
{
    PTPJointParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetPTPJointParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetPTPJointParamsService(dobot::GetPTPJointParams::Request &req, dobot::GetPTPJointParams::Response &res)
{
    PTPJointParams params;

    res.result = GetPTPJointParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool DobotAgent::SetPTPCoordinateParamsService(dobot::SetPTPCoordinateParams::Request &req, dobot::SetPTPCoordinateParams::Response &res)
{
    PTPCoordinateParams params;
    uint64_t queuedCmdIndex;

    params.xyzVelocity = req.xyzVelocity;
    params.rVelocity = req.rVelocity;
    params.xyzAcceleration = req.xyzAcceleration;
    params.rAcceleration = req.rAcceleration;
    res.result = SetPTPCoordinateParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetPTPCoordinateParamsService(dobot::GetPTPCoordinateParams::Request &req, dobot::GetPTPCoordinateParams::Response &res)
{
    PTPCoordinateParams params;

    res.result = GetPTPCoordinateParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xyzVelocity = params.xyzVelocity;
        res.rVelocity = params.rVelocity;
        res.xyzAcceleration = params.xyzAcceleration;
        res.rAcceleration = params.rAcceleration;
    }

    return true;
}

bool DobotAgent::SetPTPJumpParamsService(dobot::SetPTPJumpParams::Request &req, dobot::SetPTPJumpParams::Response &res)
{
    PTPJumpParams params;
    uint64_t queuedCmdIndex;

    params.jumpHeight = req.jumpHeight;
    params.zLimit = req.zLimit;
    res.result = SetPTPJumpParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetPTPJumpParamsService(dobot::GetPTPJumpParams::Request &req, dobot::GetPTPJumpParams::Response &res)
{
    PTPJumpParams params;

    res.result = GetPTPJumpParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.jumpHeight = params.jumpHeight;
        res.zLimit = params.zLimit;
    }

    return true;
}

bool DobotAgent::SetPTPCommonParamsService(dobot::SetPTPCommonParams::Request &req, dobot::SetPTPCommonParams::Response &res)
{
    PTPCommonParams params;
    uint64_t queuedCmdIndex;

    params.velocityRatio = req.velocityRatio;
    params.accelerationRatio = req.accelerationRatio;
    res.result = SetPTPCommonParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetPTPCommonParamsService(dobot::GetPTPCommonParams::Request &req, dobot::GetPTPCommonParams::Response &res)
{
    PTPCommonParams params;

    res.result = GetPTPCommonParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.velocityRatio = params.velocityRatio;
        res.accelerationRatio = params.accelerationRatio;
    }

    return true;
}

bool DobotAgent::SetPTPCmdService(dobot::SetPTPCmd::Request &req, dobot::SetPTPCmd::Response &res)
{
    PTPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.ptpMode = req.ptpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.r = req.r;
    res.result = SetPTPCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void DobotAgent::InitPTPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetPTPJointParams", &DobotAgent::SetPTPJointParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetPTPJointParams", &DobotAgent::GetPTPJointParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetPTPCoordinateParams", &DobotAgent::SetPTPCoordinateParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetPTPCoordinateParams", &DobotAgent::GetPTPCoordinateParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetPTPJumpParams", &DobotAgent::SetPTPJumpParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetPTPJumpParams", &DobotAgent::GetPTPJumpParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetPTPCommonParams", &DobotAgent::SetPTPCommonParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetPTPCommonParams", &DobotAgent::GetPTPCommonParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetPTPCmd", &DobotAgent::SetPTPCmdService, this);
    serverVec.push_back(server);
}

/*
 * CP
 */
#include "dobot/SetCPParams.h"
#include "dobot/GetCPParams.h"
#include "dobot/SetCPCmd.h"

bool DobotAgent::SetCPParamsService(dobot::SetCPParams::Request &req, dobot::SetCPParams::Response &res)
{
    CPParams params;
    uint64_t queuedCmdIndex;

    params.planAcc = req.planAcc;
    params.juncitionVel = req.junctionVel;
    params.acc = req.acc;
    params.realTimeTrack = req.realTimeTrack;
    res.result = SetCPParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetCPParamsService(dobot::GetCPParams::Request &req, dobot::GetCPParams::Response &res)
{
    CPParams params;

    res.result = GetCPParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.planAcc = params.planAcc;
        res.junctionVel = params.juncitionVel;
        res.acc = params.acc;
        res.realTimeTrack = params.realTimeTrack;
    }

    return true;
}

bool DobotAgent::SetCPCmdService(dobot::SetCPCmd::Request &req, dobot::SetCPCmd::Response &res)
{
    CPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.cpMode = req.cpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.velocity = req.velocity;

    res.result = SetCPCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void DobotAgent::InitCPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetCPParams", &DobotAgent::SetCPParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetCPParams", &DobotAgent::GetCPParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetCPCmd", &DobotAgent::SetCPCmdService, this);
    serverVec.push_back(server);
}

/*
 * ARC
 */
#include "dobot/SetARCParams.h"
#include "dobot/GetARCParams.h"
#include "dobot/SetARCCmd.h"

bool DobotAgent::SetARCParamsService(dobot::SetARCParams::Request &req, dobot::SetARCParams::Response &res)
{
    ARCParams params;
    uint64_t queuedCmdIndex;

    params.xyzVelocity = req.xyzVelocity;
    params.rVelocity = req.rVelocity;
    params.xyzAcceleration = req.xyzAcceleration;
    params.rAcceleration = req.rAcceleration;
    res.result = SetARCParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetARCParamsService(dobot::GetARCParams::Request &req, dobot::GetARCParams::Response &res)
{
    ARCParams params;

    res.result = GetARCParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xyzVelocity = params.xyzVelocity;
        res.rVelocity = params.rVelocity;
        res.xyzAcceleration = params.xyzAcceleration;
        res.rAcceleration = params.rAcceleration;
    }

    return true;
}

bool DobotAgent::SetARCCmdService(dobot::SetARCCmd::Request &req, dobot::SetARCCmd::Response &res)
{
    ARCCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.cirPoint.x = req.x1;
    cmd.cirPoint.y = req.y1;
    cmd.cirPoint.z = req.z1;
    cmd.cirPoint.r = req.r1;
    cmd.toPoint.x = req.x2;
    cmd.toPoint.y = req.y2;
    cmd.toPoint.z = req.z2;
    cmd.toPoint.r = req.r2;

    res.result = SetARCCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void DobotAgent::InitARCServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetARCParams", &DobotAgent::SetARCParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetARCParams", &DobotAgent::GetARCParamsService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetARCCmd", &DobotAgent::SetARCCmdService, this);
    serverVec.push_back(server);
}

/*
 * WAIT
 */
#include "dobot/SetWAITCmd.h"

bool DobotAgent::SetWAITCmdService(dobot::SetWAITCmd::Request &req, dobot::SetWAITCmd::Response &res)
{
    WAITCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.timeout = req.timeout;
    res.result = SetWAITCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void DobotAgent::InitWAITServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetWAITCmd", &DobotAgent::SetWAITCmdService, this);
    serverVec.push_back(server);
}

/*
 * TRIG
 */
#include "dobot/SetTRIGCmd.h"

bool DobotAgent::SetTRIGCmdService(dobot::SetTRIGCmd::Request &req, dobot::SetTRIGCmd::Response &res)
{
    TRIGCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.address = req.address;
    cmd.mode = req.mode;
    cmd.condition = req.condition;
    cmd.threshold = req.threshold;
    res.result = SetTRIGCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void DobotAgent::InitTRIGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetTRIGCmd", &DobotAgent::SetTRIGCmdService, this);
    serverVec.push_back(server);
}

/*
 * EIO
 */
#include "dobot/SetIOMultiplexing.h"
#include "dobot/GetIOMultiplexing.h"
#include "dobot/SetIODO.h"
#include "dobot/GetIODO.h"
#include "dobot/SetIOPWM.h"
#include "dobot/GetIOPWM.h"
#include "dobot/GetIODI.h"
#include "dobot/GetIOADC.h"
#include "dobot/SetEMotor.h"
#include "dobot/SetColorSensor.h"
#include "dobot/GetColorSensor.h"
#include "dobot/SetInfraredSensor.h"
#include "dobot/GetInfraredSensor.h"


bool DobotAgent::SetIOMultiplexingService(dobot::SetIOMultiplexing::Request &req, dobot::SetIOMultiplexing::Response &res)
{
    IOMultiplexing ioMultiplexing;
    uint64_t queuedCmdIndex;

    ioMultiplexing.address = req.address;
    ioMultiplexing.multiplex = req.multiplex;
    res.result = SetIOMultiplexing(&ioMultiplexing, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetIOMultiplexingService(dobot::GetIOMultiplexing::Request &req, dobot::GetIOMultiplexing::Response &res)
{
    IOMultiplexing ioMultiplexing;

    ioMultiplexing.address = req.address;
    res.result = GetIOMultiplexing(&ioMultiplexing);
    if (res.result == DobotCommunicate_NoError) {
        res.multiplex = ioMultiplexing.multiplex;
    }

    return true;
}

bool DobotAgent::SetIODOService(dobot::SetIODO::Request &req, dobot::SetIODO::Response &res)
{
    IODO ioDO;
    uint64_t queuedCmdIndex;

    ioDO.address = req.address;
    ioDO.level = req.level;
    res.result = SetIODO(&ioDO, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetIODOService(dobot::GetIODO::Request &req, dobot::GetIODO::Response &res)
{
    IODO ioDO;

    ioDO.address = req.address;
    res.result = GetIODO(&ioDO);
    if (res.result == DobotCommunicate_NoError) {
        res.level = ioDO.level;
    }

    return true;
}

bool DobotAgent::SetIOPWMService(dobot::SetIOPWM::Request &req, dobot::SetIOPWM::Response &res)
{
    IOPWM ioPWM;
    uint64_t queuedCmdIndex;

    ioPWM.address = req.address;
    ioPWM.frequency = req.frequency;
    ioPWM.dutyCycle = req.dutyCycle;
    res.result = SetIOPWM(&ioPWM, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool DobotAgent::GetIOPWMService(dobot::GetIOPWM::Request &req, dobot::GetIOPWM::Response &res)
{
    IOPWM ioPWM;

    ioPWM.address = req.address;
    res.result = GetIOPWM(&ioPWM);
    if (res.result == DobotCommunicate_NoError) {
        res.frequency = ioPWM.frequency;
        res.dutyCycle = ioPWM.dutyCycle;
    }

    return true;
}

bool DobotAgent::GetIODIService(dobot::GetIODI::Request &req, dobot::GetIODI::Response &res)
{
    IODI ioDI;

    ioDI.address = req.address;
    res.result = GetIODI(&ioDI);
    if (res.result == DobotCommunicate_NoError) {
        res.level = ioDI.level;
    }

    return true;
}

bool DobotAgent::GetIOADCService(dobot::GetIOADC::Request &req, dobot::GetIOADC::Response &res)
{
    IOADC ioADC;

    ioADC.address = req.address;
    res.result = GetIOADC(&ioADC);
    if (res.result == DobotCommunicate_NoError) {
        res.value = ioADC.value;
    }

    return true;
}

bool DobotAgent::SetEMotorService(dobot::SetEMotor::Request &req, dobot::SetEMotor::Response &res)
{
    EMotor eMotor;
    uint64_t queuedCmdIndex;

    eMotor.index = req.index;
    eMotor.isEnabled = req.isEnabled;
    eMotor.speed = req.speed;
    res.result = SetEMotor(&eMotor, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

// Newly added 7th Feb
bool DobotAgent::SetColorSensorService(dobot::SetColorSensor::Request &req, dobot::SetColorSensor::Response &res)
{
    uint8_t colorPort;
    bool enable;

    colorPort = req.colorPort;
    enable = req.enable;
    res.result = SetColorSensor(enable, colorPort);
    // if (res.result == DobotCommunicate_NoError) {
    //     res.queuedCmdIndex = queuedCmdIndex;
    // }

    return true;
}

bool DobotAgent::GetColorSensorService(dobot::GetColorSensor::Request &req, dobot::GetColorSensor::Response &res)
{
    //uint8_t colorPort;
    uint8_t value_r;
    uint8_t value_g;
    uint8_t value_b;

    //colorPort = req.colorPort;
    res.result = GetColorSensor(&value_r, &value_g, &value_b);
    if (res.result == DobotCommunicate_NoError) {
        res.value_r = value_r;
        res.value_g = value_g;
        res.value_b = value_b;
    }
    return true;
}

bool DobotAgent::SetInfraredSensorService(dobot::SetInfraredSensor::Request &req, dobot::SetInfraredSensor::Response &res)
{
    uint8_t infraredPort;
    bool enable;

    infraredPort = req.infraredPort;
    enable = req.enable;
    res.result = SetInfraredSensor(enable, infraredPort);
    // if (res.result == DobotCommunicate_NoError) {
    //     res.queuedCmdIndex = queuedCmdIndex;
    // }

    return true;
}

bool DobotAgent::GetInfraredSensorService(dobot::GetInfraredSensor::Request &req, dobot::GetInfraredSensor::Response &res)
{
    uint8_t infraredPort;
    uint8_t value;

    infraredPort = req.infraredPort;
    res.result = GetInfraredSensor(infraredPort, &value);
    if (res.result == DobotCommunicate_NoError) {
        res.value = value;
    }

    return true;
}


// bool GetInfraredSensorService(dobot::GetInfraredSensor::Request &req, dobot::GetInfraredSensor::Response &res)
// {
//     uint8_t infraredPort;
//     uint8_t value;

//     infraredPort = req.infraredPort;
//     res.result = GetInfaredSensor(infraredPort, &value);
//     if (res.result == DobotCommunicate_NoError) {
//         res.value = value;
//     }
//     return true;
// }


void DobotAgent::InitEIOServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetIOMultiplexing", &DobotAgent::SetIOMultiplexingService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetIOMultiplexing", &DobotAgent::GetIOMultiplexingService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetIODO", &DobotAgent::SetIODOService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetIODO", &DobotAgent::GetIODOService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetIOPWM", &DobotAgent::SetIOPWMService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetIOPWM", &DobotAgent::GetIOPWMService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetIODI",&DobotAgent::GetIODIService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetIOADC", &DobotAgent::GetIOADCService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetEMotor", &DobotAgent::SetEMotorService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetColorSensor", &DobotAgent::SetColorSensorService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetColorSensor", &DobotAgent::GetColorSensorService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetInfraredSensor", &DobotAgent::SetInfraredSensorService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/GetInfraredSensor", &DobotAgent::GetInfraredSensorService, this);
    serverVec.push_back(server);
}

/*
 * Queued command control
 */
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdStopExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/SetQueuedCmdClear.h"
//#include "dobot/GetQueuedCmdCurrentIndex.h"

bool DobotAgent::SetQueuedCmdStartExecService(dobot::SetQueuedCmdStartExec::Request &req, dobot::SetQueuedCmdStartExec::Response &res)
{
    res.result = SetQueuedCmdStartExec();

    return true;
}

bool DobotAgent::SetQueuedCmdStopExecService(dobot::SetQueuedCmdStopExec::Request &req, dobot::SetQueuedCmdStopExec::Response &res)
{
    res.result = SetQueuedCmdStopExec();

    return true;
}

bool DobotAgent::SetQueuedCmdForceStopExecService(dobot::SetQueuedCmdForceStopExec::Request &req, dobot::SetQueuedCmdForceStopExec::Response &res)
{
    res.result = SetQueuedCmdForceStopExec();

    return true;
}

bool DobotAgent::SetQueuedCmdClearService(dobot::SetQueuedCmdClear::Request &req, dobot::SetQueuedCmdClear::Response &res)
{
    res.result = SetQueuedCmdClear();

    return true;
}

// bool GetQueuedCmdCurrentIndexService(dobot::GetQueuedCmdCurrentIndex::Request &req, dobot::GetQueuedCmdCurrentIndex::Response &res)
// {
//     uint64_t index;

//     res.result = GetQueuedCmdCurrentIndex(&index);
//     if (res.result == DobotCommunicate_NoError) {
//         res.queuedCmdIndex = index;
//     }

//     return true;
// }

void DobotAgent::InitQueuedCmdServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotAgent/SetQueuedCmdStartExec", &DobotAgent::SetQueuedCmdStartExecService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetQueuedCmdStopExec", &DobotAgent::SetQueuedCmdStopExecService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetQueuedCmdForceStopExec", &DobotAgent::SetQueuedCmdForceStopExecService, this);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotAgent/SetQueuedCmdClear", &DobotAgent::SetQueuedCmdClearService, this);
    serverVec.push_back(server);
    //server = n.advertiseService("/DobotAgent/GetQueuedCmdCurrentIndex", GetQueuedCmdCurrentIndexService);
    //serverVec.push_back(server);
}

void DobotAgent::Initialize(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    InitCmdTimeoutServices(n, serverVec);
    InitDeviceInfoServices(n, serverVec);
    InitPoseServices(n, serverVec);
    InitAlarmsServices(n, serverVec);
    InitHOMEServices(n, serverVec);
    InitEndEffectorServices(n, serverVec);
    InitJOGServices(n, serverVec);
    InitPTPServices(n, serverVec);
    InitCPServices(n, serverVec);
    InitARCServices(n, serverVec);
    InitWAITServices(n, serverVec);
    InitTRIGServices(n, serverVec);
    InitEIOServices(n, serverVec);
    InitQueuedCmdServices(n, serverVec);
}