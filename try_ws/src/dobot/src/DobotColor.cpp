#include "ros/ros.h"
#include "std_msgs/String.h"
#include "DobotDll.h"
#include "dobot/GetColorSensor.h"
#include "dobot/SetColorSensor.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>
#include <std_msgs/UInt8.h>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "DobotColor");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::UInt8>("color", 1000);
  ros::ServiceClient client;
  //ros::ServiceClient client1;

  client = n.serviceClient<dobot::SetColorSensor>("/DobotServer/SetColorSensor");
  dobot::SetColorSensor srvcs;
  srvcs.request.colorPort = 2; //GP4
  srvcs.request.enable = 1;
  client.call(srvcs);

  client = n.serviceClient<dobot::GetColorSensor>("/DobotServer/GetColorSensor");
  dobot::GetColorSensor srvgcs;

  ros::Rate loop_rate(5);

  //int count = 0;

  while (ros::ok())
  {
    client.call(srvgcs);

    std_msgs::UInt8 msg;

    msg.data = srvgcs.response.value_r*100 + srvgcs.response.value_g*10 + srvgcs.response.value_b;

    ROS_INFO("result: %d", msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    //++count;
  }    

    return 0;
}