#include <teleop_control/teleop_flight_stick.h>

#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
   std::string node_name("teleop_flight_stick_device");

   ros::init(argc, argv, node_name);
   ros::NodeHandle node_handle;

   Teleop::FlightStick teleop_flight_stick(&node_handle, node_name);

   teleop_flight_stick.processMsgs();

   return 0;
}
