#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include "dobot/SetEMotor.h"
#include "DobotDll.h"
#include "ConveyorBeltAgent.h"
using namespace std;

ConveyorBeltAgent::ConveyorBeltAgent(){
	ros::NodeHandle pn("~");
	//Initialize();
}

ConveyorBeltAgent::~ConveyorBeltAgent(){
}

void ConveyorBeltAgent::Initialize(ros::NodeHandle &nh){
	//ros::NodeHandle nh("~");

	ros::ServiceClient conveyorInit;
    conveyorInit = nh.serviceClient<dobot::SetEMotor>("/DobotAgent/SetEMotor");
    dobot::SetEMotor srvm;
    EMotor eMotor;
    uint64_t queuedCmdIndex;
    
    // Start the conveyor belt
    ROS_INFO("Start.");
    srvm.request.index = 0;    
    srvm.request.speed = 10000;
    srvm.request.isEnabled = 1;
    srvm.request.isQueued = 0;
    conveyorInit.call(srvm);
    ROS_INFO("Queued command index: %ld",srvm.response.queuedCmdIndex);
}