#include <teleop_control/teleop_control.h>

#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
   std::string node_name("teleop_control_node");

   ros::init(argc, argv, node_name);
   ros::NodeHandle node_handle;

   Teleop::ControlNode teleop_control_node(&node_handle, node_name);

   teleop_control_node.processMsgs();

   return 0;
}
