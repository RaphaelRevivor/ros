#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include "std_msgs/Float32MultiArray.h"
#include "DobotDll.h"
#include "DobotAgent.h"
using namespace std;

int main(int argc, char **argv)
{
   
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            ROS_ERROR("Dobot not found!");
            return -2;
        break;
        case DobotConnect_Occupied:
            ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
            return -3;
        break;
        default:
        break;
    }
    ros::init(argc, argv, "DobotAgent");
    ros::NodeHandle n;

    // Initialization
    std::vector<ros::ServiceServer> serverVec;
    DobotAgent dobot_agent;
    dobot_agent.Initialize(n, serverVec);
    ROS_INFO("Dobot service running...");

    ros::Publisher chatter_infrared = n.advertise<std_msgs::UInt8>("infrared", 1000);

    uint8_t infraredPort = 1; // default: GP2
    bool enable = 1; //default: enable
    uint8_t value = 0;
    uint8_t connection;
    std_msgs::UInt8 msg;

    // for conveyor belt
    uint8_t old_infrared = 0;

    // Enable infrared sensor
    SetInfraredSensor(enable, infraredPort);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        uint8_t old_infrared = msg.data;

        connection = GetInfraredSensor(infraredPort, &value);
        if (connection == DobotCommunicate_NoError) {
            msg.data = value;
        }

        ROS_INFO("Infrared: %d", msg.data);

        chatter_infrared.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    ROS_INFO("Dobot service exiting...");

    // Disconnect Dobot
    DisconnectDobot();

    return 0;
}