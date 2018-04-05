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

#include "MainAgent.h"
using namespace std;

// Global pointers
//ros::ServiceClient *clientPtr;
//ros::ServiceClient *clientPtr0; 
//ros::ServiceClient *clientPtr1; 
ros::ServiceClient *clientPtr2; 
ros::ServiceClient *clientPtr3; 
ros::ServiceClient *clientPtr4; 
ros::ServiceClient *clientPtr5;
ros::ServiceClient *clientPtr6; 

// Counters for different colors
int RedCount = 0;
int BlueCount = 0;
int GreenCount = 0;

// Other global variables
int flag = 0;
Pose cur_pose;
int async_task;
int async_infrared;


void taskCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    async_task = msg->data;
}

void run(void)
{
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        int action = async_task;
        int color = 0;
        int ready = 0;

        //ros::ServiceClient client0 = (ros::ServiceClient)*clientPtr0;
        //ros::ServiceClient client1 = (ros::ServiceClient)*clientPtr1;
        ros::ServiceClient client2 = (ros::ServiceClient)*clientPtr2;
        ros::ServiceClient client3 = (ros::ServiceClient)*clientPtr3;
        ros::ServiceClient client4 = (ros::ServiceClient)*clientPtr4;
        ros::ServiceClient client5 = (ros::ServiceClient)*clientPtr5;
        ros::ServiceClient client6 = (ros::ServiceClient)*clientPtr6;
        dobot::SetPTPCmd srvptp;
        dobot::SetEndEffectorSuctionCup srvs;
        dobot::GetPose srvcp;
        dobot::SetWAITCmd srvw;
        dobot::GetColorSensor srvgcs;
        dobot::SetQueuedCmdClear srvqc;

        client4.call(srvcp);
        cur_pose.x = srvcp.response.x;
        cur_pose.y = srvcp.response.y;
        cur_pose.z = srvcp.response.z;
        //cur_pose.r = srvcp.response.r;

        // if the arm goes back to the initial location, then ready
        ready = (abs(cur_pose.x - INIT_X)<0.1) && (abs(cur_pose.y - INIT_Y)<0.1) && (abs(cur_pose.z - INIT_Z)<0.1);
        ROS_INFO("Infrared: [%d]", action);
        ROS_INFO("InitialPos: [%d]", ready);
        ROS_INFO("Flag: [%d]", flag);

        if (ready == 1)
            flag = 0;

        if(ready == 1 && action == 1 && flag == 0) //request service from the client
        {
            ROS_INFO("Box detected!");
            
            // Jump to grab location
            srvptp.request.ptpMode = 2;
            srvptp.request.x = GRAB_X; // PTP mode = 0, door shape
            srvptp.request.y = GRAB_Y;
            srvptp.request.z = GRAB_Z;
            //srvptp.request.r = cur_pose.r;
            client3.call(srvptp);
            //ros::service::waitForService("/DobotAgent/SetPTPCmd", -1);

            // Suctioncup on
            srvs.request.enableCtrl = 1;
            srvs.request.suck = 1;
            srvs.request.isQueued = 1;
            client2.call(srvs);
            //ros::service::waitForService("/DobotAgent/SetEndEffectorSuctionCup", -1);

            // Jump to color sensor location
            srvptp.request.ptpMode = 0;
            srvptp.request.x = COLOR_SENSOR_X; // PTP mode = 0
            srvptp.request.y = COLOR_SENSOR_Y;
            srvptp.request.z = COLOR_SENSOR_Z;
            //srvptp.request.r = cur_pose.r;
            client3.call(srvptp);
            //ros::service::waitForService("/DobotAgent/SetPTPCmd", -1);
            ROS_INFO("Jumped to color sensor.");

            // Delay
            srvw.request.timeout = 1500;
            client5.call(srvw);
            //ros::service::waitForService("/DobotAgent/SetWAITCmd", -1);
            //Get location; wait until reach the color sensor location
            ROS_INFO("Waiting...");

            for(;;)
            {
                client4.call(srvcp);
                cur_pose.x = srvcp.response.x;
                cur_pose.y = srvcp.response.y;
                cur_pose.z = srvcp.response.z;
                //cur_pose.r = srvcp.response.r;

                // if the arm goes to the color sensor location, then ready
                if ((abs(cur_pose.x - COLOR_SENSOR_X)<0.1) && (abs(cur_pose.y - COLOR_SENSOR_Y)<0.1) && (abs(cur_pose.z - COLOR_SENSOR_Z)<0.1))
                    break;
            }

            // Get color readings
            client6.call(srvgcs);
            color = srvgcs.response.value_r*100 + srvgcs.response.value_g*10 + srvgcs.response.value_b;
            ROS_INFO("Color reading received.");

            // Sorting
            srvptp.request.ptpMode = 0;
            switch(color)
            {
                case 100:
                    RedCount++;
                    ROS_INFO("Red: %d", RedCount);      
                    srvptp.request.x = PLACE_X; 
                    srvptp.request.y = PLACE_Y - RedCount * PLACE_INT;
                    srvptp.request.z = PLACE_Z;
                    //srvptp.request.r = cur_pose.r;          
                    break;
                case 1:
                    BlueCount++;
                    ROS_INFO("Blue: %d", BlueCount);   
                    srvptp.request.x = PLACE_X - PLACE_INT; 
                    srvptp.request.y = PLACE_Y - BlueCount * PLACE_INT;
                    srvptp.request.z = PLACE_Z;
                    //srvptp.request.r = cur_pose.r;             
                    break;
                case 10:
                    GreenCount++;
                    srvptp.request.x = PLACE_X - 2*PLACE_INT;
                    srvptp.request.y = PLACE_Y - GreenCount * PLACE_INT;
                    srvptp.request.z = PLACE_Z;
                    //srvptp.request.r = cur_pose.r;
                    ROS_INFO("Green: %d", GreenCount);
                    break;
                default:
                    srvptp.request.x = UNKNOWN_X;
                    srvptp.request.y = UNKNOWN_Y;
                    srvptp.request.z = UNKNOWN_Z;
                    ROS_INFO("Unknown: %d", color);
                    break;
            }
            int return_result = client3.call(srvptp);
            ROS_INFO("Placing called: %d", return_result);

            // SuctionCup off
            srvs.request.enableCtrl = 1;
            srvs.request.suck = 0;
            srvs.request.isQueued = 1;
            client2.call(srvs);
            ROS_INFO("Suchtion off.");

            // Jump to initial location
            srvptp.request.ptpMode = 0;
            srvptp.request.x = INIT_X; // PTP mode = 0
            srvptp.request.y = INIT_Y;
            srvptp.request.z = INIT_Z;
            //srvptp.request.r = cur_pose.r;
            int return_value = client3.call(srvptp);
            ROS_INFO("Moving to initial place: %d.", return_value);

            for(;;)
            {
                client4.call(srvcp);
                cur_pose.x = srvcp.response.x;
                cur_pose.y = srvcp.response.y;
                cur_pose.z = srvcp.response.z;
                //cur_pose.r = srvcp.response.r;
                // if the arm goes to the color sensor location, then ready
                if ((abs(cur_pose.x - INIT_X)<0.1) && (abs(cur_pose.y - INIT_Y)<0.1) && (abs(cur_pose.z - INIT_Z)<0.1))
                    break;
            }
            ROS_INFO("Jump to initial location.");
            
            // Only response to the first detection
            flag = 1;
        }
        ros::spinOnce();

        loop_rate.sleep();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_belt_agent");
    ros::NodeHandle n;

    MainAgent main_agent;
    main_agent.Initialize(n);
    ROS_INFO("Main agent is ready!");

    //ros::ServiceClient client;
    //ros::ServiceClient client0;
    //ros::ServiceClient client1;
    ros::ServiceClient client2;
    ros::ServiceClient client3;
    ros::ServiceClient client4;
    ros::ServiceClient client5;
    ros::ServiceClient client6;

    //clientPtr0 = &client0;
    //clientPtr1 = &client1;
    clientPtr2 = &client2;
    clientPtr3 = &client3;
    clientPtr4 = &client4;
    clientPtr5 = &client5;
    clientPtr6 = &client6;

    //client0 = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotAgent/SetQueuedCmdClear");
    //client1 = n.serviceClient<dobot::SetEMotor>("/DobotAgent/SetEMotor");
    client2 = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotAgent/SetEndEffectorSuctionCup");
    client3 = n.serviceClient<dobot::SetPTPCmd>("/DobotAgent/SetPTPCmd");
    client4 = n.serviceClient<dobot::GetPose>("/DobotAgent/GetPose");
    client5 = n.serviceClient<dobot::SetWAITCmd>("/DobotAgent/SetWAITCmd");
    client6 = n.serviceClient<dobot::GetColorSensor>("/DobotAgent/GetColorSensor");

    ros::Subscriber Receiver; 

    Receiver = n.subscribe("task", 1000, taskCallback);

    run();

    return 0;
}