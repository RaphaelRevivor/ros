// TODO: delete unecessary header files listed below
#ifndef MAINAGENT_H
#define MAINAGENT_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"
#include "DobotDll.h"
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>

#include "dobot/SetWAITCmd.h"
#include "dobot/GetPose.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/SetEndEffectorSuctionCup.h"
#include "dobot/SetColorSensor.h"
#include "dobot/GetColorSensor.h"
#include "dobot/SetInfraredSensor.h"
#include "dobot/GetInfraredSensor.h"

// Dobot key location definitions
#define GRAB_X 265.31
#define GRAB_Y 172.18
#define GRAB_Z 16.29
//#define GRAB_Z 30
#define COLOR_SENSOR_X 193.92
#define COLOR_SENSOR_Y 130.36
#define COLOR_SENSOR_Z 35
#define INIT_X 265.31
#define INIT_Y 172.18
#define INIT_Z 35
#define PLACE_X 140.31
#define PLACE_Y -151.79
#define PLACE_Z -40.21
#define PLACE_INT 45
#define UNKNOWN_X 265.31
#define UNKNOWN_Y 130.36
#define UNKNOWN_Z 22

// Convey belt contant definitions
#define STEP_PER_CRICLE 360.0 / 1.8 * 10.0 * 16.0
#define MM_PER_CRICLE 3.1415926535898 * 36.0

// Max function
#define MAX(X,Y) (((X)>(Y))?(X):(Y))

class MainAgent{
public:
	MainAgent();
	virtual ~MainAgent();

//private:
	int Initialize(ros::NodeHandle &n);
};

#endif
