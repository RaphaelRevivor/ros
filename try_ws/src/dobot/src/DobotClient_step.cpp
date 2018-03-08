#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"
#include "DobotDll.h"

#include "dobot/SetWAITCmd.h"
#include "dobot/GetPose.h"

#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/SetEMotor.h"
#include "dobot/SetEndEffectorSuctionCup.h"
#include "dobot/SetColorSensor.h"
#include "dobot/GetColorSensor.h"
#include "dobot/SetInfraredSensor.h"
#include "dobot/GetInfraredSensor.h"

// Dobot key location definitions
#define GRAB_X 276.55
#define GRAB_Y 172.83
#define GRAB_Z 15.85
//#define GRAB_Z 30
#define COLOR_SENSOR_X 195.23
#define COLOR_SENSOR_Y 111.36
#define COLOR_SENSOR_Z 32.45
#define INIT_X 240
#define INIT_Y 172.83
#define INIT_Z 32.45
#define PLACE_X 100
#define PLACE_Y 100
#define PLACE_Z 100
#define PLACE_INT 45

// Convey belt contant definitions
#define STEP_PER_CRICLE 360.0 / 1.8 * 10.0 * 16.0
#define MM_PER_CRICLE 3.1415926535898 * 36.0

// Max function
#define MAX(X,Y) (((X)>(Y))?(X):(Y))


// 10th Feb Todo: add global variables
// assign different clients
// put the color function to the right place
// int getColor()
// {
//   // Carry the box to the color sensor
//   client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
//   dobot::SetPTPCmd srvptp; 
//   srvptp.request.ptpMode = 2;
//   srvptp.request.x = COLOR_SENSOR_X; // PTP mode = 2
//   srvptp.request.y = COLOR_SENSOR_Y;
//   srvptp.request.z = COLOR_SENSOR_Z;
//   srvptp.request.r = Pose.r;
//   client.call(srvptp);

//   // Delay
//   client = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
//   dobot::SetWAITCmd srvw;
//   srvw.request.timeout = 1;
//   srvw.request.isQueued = 1;
//   client.call(srvw);

//   // Get color sensor data
//   client = n.serviceClient<dobot::GetColorSensor>("/DobotServer/GetColorSensor");
//   dobot::SetColorSensor srvgcs;
//   srvgcs.request.colorPort = 2; //GP2
//   client.call(srvgcs);
//   r = srvgcs.response.value_r;
//   g = srvgcs.response.value_g;
//   b = srvgcs.response.value_b;
//   max_color = MAX(MAX(r,g),MAX(g,b));

//   // Place boxes to different places
//   if (max_color == r)
//   {
//     RedCount = RedCount + 1;
//     print('Red %d',RedCount);
//     client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
//     dobot::SetPTPCmd srvptp; 
//     srvptp.request.ptpMode = 2; // PTP mode = 2
//     srvptp.request.x = PLACE_X; 
//     srvptp.request.y = PLACE_Y - RedCount * PLACE_INT;
//     srvptp.request.z = PLACE_Z;
//     srvptp.request.r = 0;
//   }
//   else if (max_color == g)
//   {
//     GreenCount = GreenCount + 1;
//     print('Green %d',GreenCount);
//     client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
//     dobot::SetPTPCmd srvptp; 
//     srvptp.request.ptpMode = 2; // PTP mode = 2
//     srvptp.request.x = PLACE_X + PLACE_INT; 
//     srvptp.request.y = PLACE_Y - GreenCount * PLACE_INT;
//     srvptp.request.z = PLACE_Z;
//     srvptp.request.r = 0;
//   }
//   else
//   {
//     BlueCount = BlueCount + 1;
//     print('Blue %d',BlueCount);
//     client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
//     dobot::SetPTPCmd srvptp; 
//     srvptp.request.ptpMode = 2; // PTP mode = 2
//     srvptp.request.x = PLACE_X + 2 * PLACE_INT; 
//     srvptp.request.y = PLACE_Y - BlueCount * PLACE_INT;
//     srvptp.request.z = PLACE_Z;
//     srvptp.request.r = 0;
//   }

//   // Suction cup off
//   client1 = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
//   dobot::SetEndEffectorSuctionCup srvs;
//   srvs.request.enableCtrl = 1;
//   srvs.request.suck = 0;
//   srvs.request.isQueued = 1;       
//   client1.call(srvs);

//   // Delay
//   client = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
//   dobot::SetWAITCmd srvw;
//   srvw.request.timeout = 1;
//   srvw.request.isQueued = 1;
//   client.call(srvw);

//   return 0;
// }
ros::ServiceClient *clientPtr; //pointer for a client
ros::ServiceClient *clientPtr1; 
ros::ServiceClient *clientPtr2; 
ros::ServiceClient *clientPtr3; 
ros::ServiceClient *clientPtr4; 
ros::ServiceClient *clientPtr5; 

// Parameters for the color identification
uint8_t r,g,b,max_color;
//r = g = b = 0;

// Counter for different colors
int RedCount = 0;
int BlueCount = 0;
int GreenCount = 0;
//RedCount = BlueCount = GreenCount = 0;
// int toggle = 0;


//int busy = 0;
//int cnt = 0;
//int action = 0;
//int old_action = 0;
//int ready = 0;
int flag = 0;
Pose cur_pose;
int async_infrared;
int async_color;


void chatterCallback(const std_msgs::UInt8::ConstPtr& msg)
{
	async_infrared = msg->data;
}

void colorCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    async_color = msg->data;
}

int routine(int sync_infrared, int sync_color)
{
	int action = 0;
    int color = sync_color;
	int ready = 0;

	ros::ServiceClient client2 = (ros::ServiceClient)*clientPtr2;
    ros::ServiceClient client3 = (ros::ServiceClient)*clientPtr3;
    ros::ServiceClient client4 = (ros::ServiceClient)*clientPtr4;
    ros::ServiceClient client5 = (ros::ServiceClient)*clientPtr5;
    dobot::SetPTPCmd srvptp;
    dobot::SetEndEffectorSuctionCup srvs;
    dobot::GetPose srvcp;
    dobot::SetWAITCmd srvw;    

    client4.call(srvcp);
    cur_pose.x = srvcp.response.x;
    cur_pose.y = srvcp.response.y;
    cur_pose.z = srvcp.response.z;
    //cur_pose.r = srvcp.response.r;

    // if the arm goes back to the initial location, then ready
    ready = (abs(cur_pose.x - INIT_X)<0.1) && (abs(cur_pose.y - INIT_Y)<0.1) && (abs(cur_pose.z - INIT_Z)<0.1);
    //old_action = action;
    //ROS_INFO("I heard: [%d]", msg->data);
    if (ready == 0)
		flag = 0;

	action = sync_infrared;

	//ROS_INFO("action: [%d]", action);


	if(ready == 1 && action == 1 && flag == 0) //request service from the client
    {
    	ROS_INFO("Box detected!");
    	
	    // Jump to grab location
	    srvptp.request.ptpMode = 0;
	    srvptp.request.x = GRAB_X; // PTP mode = 0, door shape
	    srvptp.request.y = GRAB_Y;
	    srvptp.request.z = GRAB_Z;
	    srvptp.request.r = cur_pose.r;
	    client3.call(srvptp);

	    // Suctioncup on
        srvs.request.enableCtrl = 1;
    	srvs.request.suck = 1;
    	srvs.request.isQueued = 1;
        client2.call(srvs);

        // Jump to color sensor location
	    srvptp.request.ptpMode = 0;
	    srvptp.request.x = COLOR_SENSOR_X; // PTP mode = 0
	    srvptp.request.y = COLOR_SENSOR_Y;
	    srvptp.request.z = COLOR_SENSOR_Z;
	    srvptp.request.r = cur_pose.r;
	    client3.call(srvptp);

        // Delay
        srvw.request.timeout = 1;
        client5.call(srvw);

        // Get color reading
        switch(color)
        {
            case 100:
                RedCount++;
                ROS_INFO("Red: %d", RedCount);                
                break;
            case 1:
                BlueCount++;
                ROS_INFO("Blue: %d", BlueCount);                
                break;
            case 10:
                GreenCount++;
                ROS_INFO("Green: %d", BlueCount);
                break;
            default:
                break;
        }

	    // SuctionCup off
	    srvs.request.enableCtrl = 1;
    	srvs.request.suck = 0;
    	srvs.request.isQueued = 1;
        client2.call(srvs);

        // Jump to initial location
	    srvptp.request.ptpMode = 0;
	    srvptp.request.x = INIT_X; // PTP mode = 0
	    srvptp.request.y = INIT_Y;
	    srvptp.request.z = INIT_Z;
	    srvptp.request.r = cur_pose.r;
	    client3.call(srvptp);
	    
		// Only response to the first detection
	    flag = 1;
    }

    return 0;
}

void run(void)
{
	int sync_infrared;
    int sync_color;
  	ros::Rate loop_rate(5);

	while (ros::ok())
	{
		sync_infrared = async_infrared;
        sync_color = async_color;

  		routine(sync_infrared, sync_color);  	

		ros::spinOnce();

		loop_rate.sleep();
	}   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    ros::ServiceClient client;
    ros::ServiceClient client1;
    ros::ServiceClient client2;
    ros::ServiceClient client3;
    ros::ServiceClient client4;
    ros::ServiceClient client5;
    ros::ServiceClient client6;

    clientPtr = &client;
    clientPtr1 = &client1;
    clientPtr2 = &client2;
    clientPtr3 = &client3;
    clientPtr4 = &client4;
    clientPtr5 = &client5;

    // SetCmdTimeout
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    // Clear the command queue
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);

    // Start running the command queue
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);

    // Get device version information
    client = n.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // Set end effector parameters
    client = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);

    // Set PTP joint parameters
    do {
        client = n.serviceClient<dobot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
        dobot::SetPTPJointParams srv;

        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        client.call(srv);
    } while (0);

    // Set PTP coordinate parameters
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);

    // Set PTP jump parameters
    do {
        client = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // Set PTP common parameters
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv;
 
        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);


    // Set end effector parameters
    client = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srvep;
    srvep.request.xBias = 59.7;
    srvep.request.yBias = 0;
    srvep.request.zBias = 0;
    srvep.request.isQueued = 1;
    client.call(srvep);

    // Set color sensor
    client = n.serviceClient<dobot::SetColorSensor>("/DobotServer/SetColorSensor");
    dobot::SetColorSensor srvcs;
    srvcs.request.colorPort = 2; //GP2
    srvcs.request.enable = 1;
    client.call(srvcs);

    // Set infrared sensor
    client = n.serviceClient<dobot::SetInfraredSensor>("/DobotServer/SetInfraredSensor");
    dobot::SetInfraredSensor srvis;
    srvis.request.infraredPort = 1; //GP1
    srvis.request.enable = 1;
    client.call(srvis);

    // Get current position
    client = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
    dobot::GetPose srvcp;    
    client.call(srvcp);
    cur_pose.x = srvcp.response.x;
    cur_pose.y = srvcp.response.y;
    cur_pose.z = srvcp.response.z;
    cur_pose.r = srvcp.response.r;

    // Delay
    client = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
    dobot::SetWAITCmd srvw;
    srvw.request.timeout = 1;
    //srvw.request.isQueued = 1;
    client.call(srvw);

    // The convey belt: currently not working
    client = n.serviceClient<dobot::SetEMotor>("/DobotServer/SetEMotor");
    dobot::SetEMotor srvm;
    srvm.request.index = 0;    
    float vel = float(50) * STEP_PER_CRICLE / MM_PER_CRICLE;
    srvm.request.speed = vel;
    srvm.request.isEnabled = 1;
    srvm.request.isQueued = 0;
    client.call(srvm);
    ROS_INFO("%d",srvm.response.result);



    // PTP mode jump to the initial location: color sensor
    client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd srvptp; 
    srvptp.request.ptpMode = 2;
    srvptp.request.x = INIT_X; // PTP mode = 2
    srvptp.request.y = INIT_Y;
    srvptp.request.z = INIT_Z;
    srvptp.request.r = cur_pose.r;
    client.call(srvptp);

    // Set the suctioncup to off
    client = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
    dobot::SetEndEffectorSuctionCup srvs;
    srvs.request.enableCtrl = 1;
    srvs.request.suck = 0;
    srvs.request.isQueued = 1;       
    client.call(srvs);

    client2 = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
	client3 = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
	client4 = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
    client5 = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");


    ros::Subscriber Receiver; 
    ros::Subscriber Receiver1;

    Receiver = n.subscribe("chatter", 1000, chatterCallback);

    Receiver1 = n.subscribe("color", 1000, colorCallback);


    run();

    //ros::spinOnce();

    //ros::spin();

    return 0;
}



    //ros::init(argc, argv, "listener");


    // while (1)
    // {
    //   for(;;)
    //   {
    //     srvgis.request.infraredPort = 1; // GP1
    //     client2.call(srvgis);
    //     //ros::spinOnce();
    //     if (srvgis.response.value == 1)
    //     {
    //       break;
    //     }
    //   }
    //     //Delay
    //     //client = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
    //     //dobot::SetWAITCmd srvw;
    //     srvw.request.timeout = 0.3;
    //     //srvw.request.isQueued = 1;
    //     client4.call(srvw);
    //     //ros::spinOnce();

    //     // Stop the convey belt
    //     //client = n.serviceClient<dobot::SetEMotor>("/DobotServer/SetEMotor");
    //     //dobot::SetEMotor srvm;
    //     // vel = 0;
    //     // srvm.request.speed = vel;
    //     // srvm.request.isEnabled = 1;
    //     // srvm.request.isQueued = 0;
    //     // client5.call(srvm);

    //     // Jump to grab location
    //     //client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    //     //dobot::SetPTPCmd srvptp; 
    //     srvptp.request.ptpMode = 2;
    //     srvptp.request.x = GRAB_X; // PTP mode = 2
    //     srvptp.request.y = GRAB_Y;
    //     srvptp.request.z = GRAB_Z;
    //     srvptp.request.r = cur_pose.r;
    //     client3.call(srvptp);
    //     //ros::spinOnce();

    //     // Turn on the suctioncup
    //     srvs.request.enableCtrl = 1;
    //     srvs.request.suck = 1;
    //     srvs.request.isQueued = 1;       
    //     client1.call(srvs);
    //     //ros::spinOnce();


    //     // Turn on the convey belt again
    //     //client = n.serviceClient<dobot::SetEMotor>("/DobotServer/SetEMotor");
    //     //dobot::SetEMotor srvm;
    //     // vel = float(50) * STEP_PER_CRICLE * MM_PER_CRICLE;
    //     // srvm.request.speed = vel;
    //     // srvm.request.isEnabled = 1;
    //     // srvm.request.isQueued = 0;
    //     // client5.call(srvm);

   

    //     // Go back to the color sensor location
    //     //client = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    //     //dobot::SetPTPCmd srvptp; 
    //     srvptp.request.ptpMode = 2;
    //     srvptp.request.x = COLOR_SENSOR_X; // PTP mode = 2
    //     srvptp.request.y = COLOR_SENSOR_Y;
    //     srvptp.request.z = COLOR_SENSOR_Z;
    //     srvptp.request.r = cur_pose.r;
    //     client3.call(srvptp);
    //     //ros::spinOnce();

    //     // Color identification
    //     //getColor();
    //     srvw.request.timeout = 1;
    //     //srvw.request.isQueued = 1;
    //     client4.call(srvw);
    //     //ros::spinOnce();

    //     // Turn off the suctioncup
    //     srvs.request.enableCtrl = 1;
    //     srvs.request.suck = 0;
    //     srvs.request.isQueued = 1;       
    //     client1.call(srvs);
    //     //ros::spinOnce();     


    // }



