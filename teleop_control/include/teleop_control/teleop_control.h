#ifndef TELEOP_CONTROL_H
#define TELEOP_CONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <teleop_control_msgs/TeleopDevice.h>
#include <teleop_control_msgs/Gear.h>

namespace Teleop
{
class ControlNode
{
public:
   ControlNode(ros::NodeHandle* node_handle, const std::string& node_name);
   ~ControlNode();

   void processMsgs();

private:
   void
      teleopDeviceMsgCallback(const teleop_control_msgs::TeleopDevice::ConstPtr& teleop_device_msg);

   void gearMsgCallback(const teleop_control_msgs::Gear::ConstPtr& gear_msg);

   bool isDeadmanPressed();
   bool isTurboModeActive();

   void processGearButtonStates();

   void stopVehicle();
   void zeroMotion();

   ros::NodeHandle mNodeHandle;

   ros::Publisher       mTwistPublisher;
   geometry_msgs::Twist mCurrentTwistMsg;

   ros::Subscriber                   mTeleopDeviceSubscriber;
   teleop_control_msgs::TeleopDevice mCurrentTeleopDeviceMsg;

   ros::Subscriber           mGearSubscriber;
   teleop_control_msgs::Gear mCurrentGearMsg;

   double mScaleLinear;
   double mScaleSteering;
   double mScaleTurbo;
   double mGearScalingFactor;

   bool mIsDeadmanRequired;

   bool mIsInTurboMode;
   bool mIsTurboAllowed;
};
} // namespace Teleop

#endif // TELEOP_CONTROL_H
