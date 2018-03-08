#include "ros/ros.h"
#include "std_msgs/String.h"
#include "DobotDll.h"
#include "dobot/GetInfraredSensor.h"
#include "dobot/SetInfraredSensor.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>
#include <std_msgs/UInt8.h>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "DobotPublisher");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::UInt8>("chatter", 1000);
  ros::ServiceClient client;
  //ros::ServiceClient client1;

  client = n.serviceClient<dobot::SetInfraredSensor>("/DobotServer/SetInfraredSensor");
  dobot::SetInfraredSensor srvis;
  srvis.request.infraredPort = 1; //GP1
  srvis.request.enable = 1;
  client.call(srvis);

  client = n.serviceClient<dobot::GetInfraredSensor>("/DobotServer/GetInfraredSensor");
  dobot::GetInfraredSensor srvgis;
  srvgis.request.infraredPort = 1; //GP1


  ros::Rate loop_rate(10);

  //int count = 0;

  while (ros::ok())
  {
    client.call(srvgis);

    std_msgs::UInt8 msg;

    msg.data = srvgis.response.value;

    ROS_INFO("result: %d", msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    //++count;
  }    

    return 0;
}