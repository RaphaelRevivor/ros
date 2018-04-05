// TODO: delete unecessary header files listed below
#ifndef DOBOTAGENT_H
#define DOBOTAGENT_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include "dobot/SetEMotor.h"
#include "DobotDll.h"

// include service header file
#include "dobot/SetCmdTimeout.h"

#include "dobot/GetDeviceSN.h"
#include "dobot/SetDeviceName.h"
#include "dobot/GetDeviceName.h"
#include "dobot/GetDeviceVersion.h"

#include "dobot/GetPose.h"

#include "dobot/GetAlarmsState.h"
#include "dobot/ClearAllAlarmsState.h"

#include "dobot/SetHOMEParams.h"
#include "dobot/GetHOMEParams.h"
#include "dobot/SetHOMECmd.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/GetEndEffectorParams.h"
#include "dobot/SetEndEffectorLaser.h"
#include "dobot/GetEndEffectorLaser.h"
#include "dobot/SetEndEffectorSuctionCup.h"
#include "dobot/GetEndEffectorSuctionCup.h"
#include "dobot/SetEndEffectorGripper.h"
#include "dobot/GetEndEffectorGripper.h"

#include "dobot/SetJOGJointParams.h"
#include "dobot/GetJOGJointParams.h"
#include "dobot/SetJOGCoordinateParams.h"
#include "dobot/GetJOGCoordinateParams.h"
#include "dobot/SetJOGCommonParams.h"
#include "dobot/GetJOGCommonParams.h"
#include "dobot/SetJOGCmd.h"

#include "dobot/SetPTPJointParams.h"
#include "dobot/GetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/GetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/GetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/GetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"

#include "dobot/SetCPParams.h"
#include "dobot/GetCPParams.h"
#include "dobot/SetCPCmd.h"

#include "dobot/SetARCParams.h"
#include "dobot/GetARCParams.h"
#include "dobot/SetARCCmd.h"

#include "dobot/SetWAITCmd.h"

#include "dobot/SetTRIGCmd.h"

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

#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdStopExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/SetQueuedCmdClear.h"

class DobotAgent{
public:
	DobotAgent();
	virtual ~DobotAgent();
    void Initialize(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);

//private:
	bool SetCmdTimeoutService(dobot::SetCmdTimeout::Request &req, dobot::SetCmdTimeout::Response &res);
	void InitCmdTimeoutServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);
	
	bool GetDeviceSNService(dobot::GetDeviceSN::Request &req, dobot::GetDeviceSN::Response &res);
	bool SetDeviceNameService(dobot::SetDeviceName::Request &req, dobot::SetDeviceName::Response &res);
	bool GetDeviceNameService(dobot::GetDeviceName::Request &req, dobot::GetDeviceName::Response &res);
	bool GetDeviceVersionService(dobot::GetDeviceVersion::Request &req, dobot::GetDeviceVersion::Response &res);
    void InitDeviceInfoServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);

	bool GetPoseService(dobot::GetPose::Request &req, dobot::GetPose::Response &res);
	void InitPoseServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);
	bool GetAlarmsStateService(dobot::GetAlarmsState::Request &req, dobot::GetAlarmsState::Response &res);
	bool ClearAllAlarmsStateService(dobot::ClearAllAlarmsState::Request &req, dobot::ClearAllAlarmsState::Response &res);
	void InitAlarmsServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);
	bool SetHOMEParamsService(dobot::SetHOMEParams::Request &req, dobot::SetHOMEParams::Response &res);
	bool GetHOMEParamsService(dobot::GetHOMEParams::Request &req, dobot::GetHOMEParams::Response &res);
	bool SetHOMECmdService(dobot::SetHOMECmd::Request &req, dobot::SetHOMECmd::Response &res);
    void InitHOMEServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);

	bool SetEndEffectorParamsService(dobot::SetEndEffectorParams::Request &req, dobot::SetEndEffectorParams::Response &res);
	bool GetEndEffectorParamsService(dobot::GetEndEffectorParams::Request &req, dobot::GetEndEffectorParams::Response &res);
	bool SetEndEffectorLaserService(dobot::SetEndEffectorLaser::Request &req, dobot::SetEndEffectorLaser::Response &res);
	bool GetEndEffectorLaserService(dobot::GetEndEffectorLaser::Request &req, dobot::GetEndEffectorLaser::Response &res);
	bool SetEndEffectorSuctionCupService(dobot::SetEndEffectorSuctionCup::Request &req, dobot::SetEndEffectorSuctionCup::Response &res);
	bool GetEndEffectorSuctionCupService(dobot::GetEndEffectorSuctionCup::Request &req, dobot::GetEndEffectorSuctionCup::Response &res);
	bool SetEndEffectorGripperService(dobot::SetEndEffectorGripper::Request &req, dobot::SetEndEffectorGripper::Response &res);
	bool GetEndEffectorGripperService(dobot::GetEndEffectorGripper::Request &req, dobot::GetEndEffectorGripper::Response &res);
    void InitEndEffectorServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);


	bool SetJOGJointParamsService(dobot::SetJOGJointParams::Request &req, dobot::SetJOGJointParams::Response &res);
	bool GetJOGJointParamsService(dobot::GetJOGJointParams::Request &req, dobot::GetJOGJointParams::Response &res);
	bool SetJOGCoordinateParamsService(dobot::SetJOGCoordinateParams::Request &req, dobot::SetJOGCoordinateParams::Response &res);
	bool GetJOGCoordinateParamsService(dobot::GetJOGCoordinateParams::Request &req, dobot::GetJOGCoordinateParams::Response &res);
	bool SetJOGCommonParamsService(dobot::SetJOGCommonParams::Request &req, dobot::SetJOGCommonParams::Response &res);
	bool GetJOGCommonParamsService(dobot::GetJOGCommonParams::Request &req, dobot::GetJOGCommonParams::Response &res);
	bool SetJOGCmdService(dobot::SetJOGCmd::Request &req, dobot::SetJOGCmd::Response &res);
    void InitJOGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);

	bool SetPTPJointParamsService(dobot::SetPTPJointParams::Request &req, dobot::SetPTPJointParams::Response &res);
	bool GetPTPJointParamsService(dobot::GetPTPJointParams::Request &req, dobot::GetPTPJointParams::Response &res);
	bool SetPTPCoordinateParamsService(dobot::SetPTPCoordinateParams::Request &req, dobot::SetPTPCoordinateParams::Response &res);
	bool GetPTPCoordinateParamsService(dobot::GetPTPCoordinateParams::Request &req, dobot::GetPTPCoordinateParams::Response &res);
	bool SetPTPJumpParamsService(dobot::SetPTPJumpParams::Request &req, dobot::SetPTPJumpParams::Response &res);
	bool GetPTPJumpParamsService(dobot::GetPTPJumpParams::Request &req, dobot::GetPTPJumpParams::Response &res);
	bool SetPTPCommonParamsService(dobot::SetPTPCommonParams::Request &req, dobot::SetPTPCommonParams::Response &res);
	bool GetPTPCommonParamsService(dobot::GetPTPCommonParams::Request &req, dobot::GetPTPCommonParams::Response &res);
	bool SetPTPCmdService(dobot::SetPTPCmd::Request &req, dobot::SetPTPCmd::Response &res);
    void InitPTPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);

	bool SetCPParamsService(dobot::SetCPParams::Request &req, dobot::SetCPParams::Response &res);
	bool GetCPParamsService(dobot::GetCPParams::Request &req, dobot::GetCPParams::Response &res);
	bool SetCPCmdService(dobot::SetCPCmd::Request &req, dobot::SetCPCmd::Response &res);
	void InitCPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);
	bool SetARCParamsService(dobot::SetARCParams::Request &req, dobot::SetARCParams::Response &res);
	bool GetARCParamsService(dobot::GetARCParams::Request &req, dobot::GetARCParams::Response &res);
	bool SetARCCmdService(dobot::SetARCCmd::Request &req, dobot::SetARCCmd::Response &res);
    void InitARCServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);

	bool SetWAITCmdService(dobot::SetWAITCmd::Request &req, dobot::SetWAITCmd::Response &res);
	void InitWAITServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);
	bool SetTRIGCmdService(dobot::SetTRIGCmd::Request &req, dobot::SetTRIGCmd::Response &res);
    void InitTRIGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);

	bool SetIOMultiplexingService(dobot::SetIOMultiplexing::Request &req, dobot::SetIOMultiplexing::Response &res);
	bool GetIOMultiplexingService(dobot::GetIOMultiplexing::Request &req, dobot::GetIOMultiplexing::Response &res);
	bool SetIODOService(dobot::SetIODO::Request &req, dobot::SetIODO::Response &res);
	bool GetIODOService(dobot::GetIODO::Request &req, dobot::GetIODO::Response &res);
	bool SetIOPWMService(dobot::SetIOPWM::Request &req, dobot::SetIOPWM::Response &res);
	bool GetIOPWMService(dobot::GetIOPWM::Request &req, dobot::GetIOPWM::Response &res);
	bool GetIODIService(dobot::GetIODI::Request &req, dobot::GetIODI::Response &res);
	bool GetIOADCService(dobot::GetIOADC::Request &req, dobot::GetIOADC::Response &res);
	bool SetEMotorService(dobot::SetEMotor::Request &req, dobot::SetEMotor::Response &res);
	bool SetColorSensorService(dobot::SetColorSensor::Request &req, dobot::SetColorSensor::Response &res);
	bool GetColorSensorService(dobot::GetColorSensor::Request &req, dobot::GetColorSensor::Response &res);
	bool SetInfraredSensorService(dobot::SetInfraredSensor::Request &req, dobot::SetInfraredSensor::Response &res);
	bool GetInfraredSensorService(dobot::GetInfraredSensor::Request &req, dobot::GetInfraredSensor::Response &res);
    void InitEIOServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);

	bool SetQueuedCmdStartExecService(dobot::SetQueuedCmdStartExec::Request &req, dobot::SetQueuedCmdStartExec::Response &res);
	bool SetQueuedCmdStopExecService(dobot::SetQueuedCmdStopExec::Request &req, dobot::SetQueuedCmdStopExec::Response &res);
	bool SetQueuedCmdForceStopExecService(dobot::SetQueuedCmdForceStopExec::Request &req, dobot::SetQueuedCmdForceStopExec::Response &res);
	bool SetQueuedCmdClearService(dobot::SetQueuedCmdClear::Request &req, dobot::SetQueuedCmdClear::Response &res);
    void InitQueuedCmdServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec);

};

#endif
