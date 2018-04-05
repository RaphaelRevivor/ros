#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include "dobot/SetEMotor.h"
#include "DobotDll.h"
#include "ConveyorBeltAgent.h"
using namespace std;

//int async_infrared[5] = {0,0,0,0,0};
int temp_infrared = 0;
int old_sum = 0;

void infraredCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    // for (int i=4; i>0; i--)
    // {
    //     async_infrared[i] = async_infrared[i-1];
    // }
    // async_infrared[0] = msg->data;
    old_sum = temp_infrared;
    temp_infrared = msg->data;
    ROS_INFO("%d",msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conveyor_belt_agent");
    ros::NodeHandle nh;

    ConveyorBeltAgent conveyor_belt_agent;
    conveyor_belt_agent.Initialize(nh);
    ROS_INFO("Conveyor belt is ready!");

    ros::ServiceClient client;
    client = nh.serviceClient<dobot::SetEMotor>("/DobotAgent/SetEMotor");
    dobot::SetEMotor srvm;
    EMotor eMotor;
    uint64_t queuedCmdIndex;
    
    // Start the conveyor belt
    ROS_INFO("Start.");
    srvm.request.index = 0;    
    srvm.request.speed = 10000;
    srvm.request.isEnabled = 1;
    srvm.request.isQueued = 0;
    client.call(srvm);

    // Subscriber for the infrared sensor readings
    ros::Subscriber Receiver; 
    Receiver = nh.subscribe("infrared", 1000, infraredCallback);    

    // Publisher for the task signal
    ros::Publisher chatter_task = nh.advertise<std_msgs::UInt8>("task", 1000);

    //int old_sum = 0;
    int sum_infrared = 0;
    std_msgs::UInt8 msg;

    ros::Rate loop_rate(10);   
    while (ros::ok())
    {
        //old_sum = sum_infrared;
        //sum_infrared = async_infrared[0] + async_infrared[1] + async_infrared[2] + async_infrared[3] + async_infrared[4];
        //if (sum_infrared == 5 && old_sum !=5) {
        if (temp_infrared == 1 && old_sum == 0) {
            msg.data = 1;
            // conveybelt stop
            ROS_INFO("Stop.");
            srvm.request.index = 0;    
            srvm.request.speed = 0;
            srvm.request.isEnabled = 1;
            srvm.request.isQueued = 1;
            client.call(srvm);
            ROS_INFO("Queued command index: %ld",srvm.response.queuedCmdIndex);
        }
        //if (sum_infrared == 0 && old_sum <5 && old_sum >0) {
        if (temp_infrared == 0 && old_sum == 1) {
            msg.data = 0;
            // conveybelt resume
            ROS_INFO("Resume.");
            srvm.request.index = 0;    
            srvm.request.speed = 10000;
            srvm.request.isEnabled = 1;
            srvm.request.isQueued = 1;
            client.call(srvm);
            ROS_INFO("Queued command index: %ld",srvm.response.queuedCmdIndex);
        }

        ROS_INFO("Task: %d", msg.data);

        chatter_task.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}