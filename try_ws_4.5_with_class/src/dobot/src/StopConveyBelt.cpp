#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/SetHOMECmd.h"
#include "dobot/GetPose.h"
#include "dobot/SetEndEffectorSuctionCup.h"
#include "dobot/SetEMotor.h"

// Convey belt contant definitions
#define STEP_PER_CRICLE 360.0 / 1.8 * 10.0 * 16.0
#define MM_PER_CRICLE 3.1415926535898 * 36.0

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    ros::ServiceClient client;

    // SetCmdTimeout
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotAgent/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotAgent isn't started yet!");
        return -1;
    }

    // Clear the command queue
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotAgent/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);

    // Start running the command queue
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotAgent/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);

    // Get device version information
    client = n.serviceClient<dobot::GetDeviceVersion>("/DobotAgent/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // The convey belt: currently not working
    client = n.serviceClient<dobot::SetEMotor>("/DobotAgent/SetEMotor");
    dobot::SetEMotor srvm;
    srvm.request.index = 0;    
    //float vel = float(100) * STEP_PER_CRICLE / MM_PER_CRICLE;
    srvm.request.speed = 10000;
    srvm.request.isEnabled = 0;
    srvm.request.isQueued = 1;
    client.call(srvm);
    ROS_INFO("%d",srvm.response.result);
    ROS_INFO("%ld", srvm.response.queuedCmdIndex);


    // Set the suctioncup to off
    client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotAgent/SetEndEffectorSuctionCup");
    dobot::SetEndEffectorSuctionCup srvs;
    srvs.request.enableCtrl = 1;
    srvs.request.suck = 0;
    srvs.request.isQueued = 1;       
    client.call(srvs);





    return 0;
}

