// TODO: delete unecessary header files listed below
#ifndef CONVEYORBELTAGENT_H
#define CONVEYORBELTAGENT_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include "dobot/SetEMotor.h"
#include "DobotDll.h"



class ConveyorBeltAgent{
public:
	ConveyorBeltAgent();
	virtual ~ConveyorBeltAgent();

//private:
	void Initialize(ros::NodeHandle &nh);
};

#endif
