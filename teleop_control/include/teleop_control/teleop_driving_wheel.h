#ifndef TELEOP_DRIVING_WHEEL_H
#define TELEOP_DRIVING_WHEEL_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <teleop_control_msgs/TeleopDevice.h>

namespace Teleop
{
class DrivingWheel
{
public:
   DrivingWheel(ros::NodeHandle* node_handle, const std::string& node_name);
   ~DrivingWheel();

   void processMsgs();

private:
   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

   double readThrottle();
   double readBrake();
   double readSteeringAngle();
   double readDeadman();
   double readHardwareInputDevice(int device_index);

   bool isAxisIndexValid(int index);
   bool isButtonIndexValid(int index);

   void brake();

   ros::NodeHandle mNodeHandle;

   ros::Subscriber  mJoySubscriber;
   sensor_msgs::Joy mCurrentJoyMsg;

   ros::Publisher                    mTeleopDevicePublisher;
   teleop_control_msgs::TeleopDevice mCurrentTeleopDeviceMsg;

   int mThrottleAxisIndex;
   int mBrakeAxisIndex;
   int mSteeringAxisIndex;
   int mDeadmanPedalAxisIndex;
};
} // namespace Teleop

#endif // TELEOP_DRIVING_WHEEL_H
