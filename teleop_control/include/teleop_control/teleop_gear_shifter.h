#ifndef TELEOP_GEAR_SHIFTER_H
#define TELEOP_GEAR_SHIFTER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <teleop_control_msgs/Gear.h>

namespace Teleop
{
class GearShifter
{
public:
   GearShifter(ros::NodeHandle* node_handle, const std::string& node_name);
   ~GearShifter();

   void processMsgs();

private:
   void joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

   uint8_t readGear();

   bool readButton(int device_index);

   bool isAxisIndexValid(int index);
   bool isButtonIndexValid(int index);

   ros::NodeHandle mNodeHandle;

   ros::Subscriber  mJoySubscriber;
   sensor_msgs::Joy mCurrentJoyMsg;

   ros::Publisher            mTeleopGearPublisher;
   teleop_control_msgs::Gear mCurrentGearMsg;

   int mReverseGearIndex;
   int mReverseTurboGearIndex;
   int mForwardGearIndex;
   int mForwardTurboGearIndex;
};
} // namespace Teleop

#endif // TELEOP_GEAR_SHIFTER_H
