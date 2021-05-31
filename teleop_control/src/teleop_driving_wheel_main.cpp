#include <teleop_control/teleop_driving_wheel.h>

#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
   std::string node_name("teleop_driving_wheel_device");

   ros::init(argc, argv, node_name);
   ros::NodeHandle node_handle;

   Teleop::DrivingWheel teleop_driving_wheel(&node_handle, node_name);

   teleop_driving_wheel.processMsgs();

   return 0;
}
