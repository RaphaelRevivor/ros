#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "DobotDll.h"

#include "dobot/SetInfraredSensor.h"
#include "dobot/GetInfraredSensor.h"

#include <sstream>
#include <std_msgs/UInt8.h>

uint8_t infraredResult = 0;
uint8_t infraredPort = 1; // default: GP1
bool enable = 1; //default: enable


// bool SetInfraredSensorService(dobot::SetInfraredSensor::Request &req, dobot::SetInfraredSensor::Response &res)
// {
//     bool enable;

//     infraredPort = req.infraredPort;
//     enable = req.enable;
//     res.result = SetInfraredSensor(enable, infraredPort);
//     // if (res.result == DobotCommunicate_NoError) {
//     //     res.queuedCmdIndex = queuedCmdIndex;
//     // }

//     return true;
// }

bool GetInfraredSensorService(dobot::GetInfraredSensor::Request &req, dobot::GetInfraredSensor::Response &res)
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


// void InitInfraredServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
// {
//     ros::ServiceServer server;

//     server = n.advertiseService("/InfraredAgent/SetInfraredSensor", SetInfraredSensorService);
//     serverVec.push_back(server);
//     server = n.advertiseService("/InfraredAgent/GetInfraredSensor", GetInfraredSensorService);
//     serverVec.push_back(server);
// }

int run(void)
{

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::UInt8>("InfraredChatter", 1000);

    uint8_t value;
    uint8_t connection;

    //SetInfraredSensor(infraredPort, enable);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::UInt8 msg;

        connection = GetInfraredSensor(infraredPort, &value);
        if (connection == DobotCommunicate_NoError) {
            infraredResult = value;
        }

        //msg.data = 1;
        msg.data = infraredResult;

        ROS_INFO("%d", msg.data);

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

int main(int argc, char **argv)
{

    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    
    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, 0, 0);

    ROS_INFO("Connect to: %s", argv[1]);

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

    ros::init(argc, argv, "InfraredAgent");

    ros::NodeHandle n;


    //std::vector<ros::ServiceServer> serverVec;

    //InitInfraredServices(n, serverVec);

    //run();

    ros::Publisher chatter_pub = n.advertise<std_msgs::UInt8>("chatter", 1000);

  // ros::ServiceClient client;
  // //ros::ServiceClient client1;

  // client = n.serviceClient<dobot::SetInfraredSensor>("/InfraredAgent/SetInfraredSensor");
  // dobot::SetInfraredSensor srvis;
  // srvis.request.infraredPort = 1; //GP1
  // srvis.request.enable = 1;
  // client.call(srvis);

  // client = n.serviceClient<dobot::GetInfraredSensor>("/InfraredAgent/GetInfraredSensor");
  // dobot::GetInfraredSensor srvgis;
  // srvgis.request.infraredPort = 1; //GP1


    uint8_t value = 0;
    uint8_t connection;

    //SetInfraredSensor(1, 1);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std_msgs::UInt8 msg;
        //client.call(srvgis);

        connection = GetInfraredSensor(infraredPort, &value);
        if (connection == DobotCommunicate_NoError) {
            infraredResult = value;
        }

        msg.data = infraredResult;

        //msg.data = 1;

        //msg.data = srvgis.response.value;


        ROS_INFO("%d", msg.data);

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}