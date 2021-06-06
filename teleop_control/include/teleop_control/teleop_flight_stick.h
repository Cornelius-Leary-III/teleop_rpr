#ifndef TELEOP_FLIGHT_STICK_H
#define TELEOP_FLIGHT_STICK_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <teleop_control_msgs/TeleopDevice.h>
#include <teleop_control_msgs/Gear.h>

namespace Teleop
{
class FlightStick
{
public:
   FlightStick(ros::NodeHandle* node_handle, const std::string& node_name);
   ~FlightStick();

   void processMsgs();

private:
   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

   double readThrottle();
   double readBrake();
   double readSteeringAngle();
   double readDeadman();
   double readHardwareInputDevice(int device_index);
   bool   readButton(int device_index);

   bool isAxisIndexValid(int index);
   bool isButtonIndexValid(int index);

   void brake();
   void setUnknownGear();

   ros::NodeHandle mNodeHandle;

   ros::Subscriber  mJoySubscriber;
   sensor_msgs::Joy mCurrentJoyMsg;

   ros::Publisher                    mTeleopDevicePublisher;
   teleop_control_msgs::TeleopDevice mCurrentTeleopDeviceMsg;

   ros::Publisher            mTeleopGearPublisher;
   teleop_control_msgs::Gear mCurrentGearMsg;

   int mThrottleAxisIndex;
   int mSteeringAxisIndex;
   int mDeadmanButtonIndex;
   int mTurboButtonIndex;
};
} // namespace Teleop

#endif // TELEOP_FLIGHT_STICK_H
