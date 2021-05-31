#include <teleop_control/teleop_gear_shifter.h>

#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
   std::string node_name("teleop_gear_shifter_device");

   ros::init(argc, argv, node_name);
   ros::NodeHandle node_handle;

   Teleop::GearShifter teleop_gear_shifter(&node_handle, node_name);

   teleop_gear_shifter.processMsgs();

   return 0;
}
