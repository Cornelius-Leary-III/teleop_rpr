#include <teleop_control/teleop_gear_shifter.h>

#include <iostream>

namespace Teleop
{
const double gTeleopDeviceMsgPublishRate   = 10.0;
const int    gReverseGearIndexDefault      = 0;
const int    gReverseTurboGearIndexDefault = 1;
const int    gForwardGearIndexDefault      = 2;
const int    gForwardTurboGearIndexDefault = 3;

GearShifter::GearShifter(ros::NodeHandle* node_handle, const std::string& node_name)
   : mNodeHandle(*node_handle),
     mCurrentJoyMsg(),
     mCurrentGearMsg(),
     mReverseGearIndex(gReverseGearIndexDefault),
     mReverseTurboGearIndex(gReverseTurboGearIndexDefault),
     mForwardGearIndex(gForwardGearIndexDefault),
     mForwardTurboGearIndex(gForwardTurboGearIndexDefault)
{
   //   std::string node_namespace = mNodeHandle.getNamespace();
   std::string node_namespace(node_name + "/");

   mNodeHandle.param(node_namespace + "gear_reverse", mReverseGearIndex, mReverseGearIndex);
   mNodeHandle.param(node_namespace + "gear_reverse_turbo",
                     mReverseTurboGearIndex,
                     mReverseTurboGearIndex);

   mNodeHandle.param(node_namespace + "gear_forward", mForwardGearIndex, mForwardGearIndex);
   mNodeHandle.param(node_namespace + "gear_forward_turbo",
                     mForwardTurboGearIndex,
                     mForwardTurboGearIndex);

   mTeleopGearPublisher = mNodeHandle.advertise<teleop_control_msgs::Gear>("gear", 1);

   mJoySubscriber = mNodeHandle.subscribe("joy", 10, &GearShifter::joyMsgCallback, this);

   mCurrentGearMsg.header.frame_id = "";
   mCurrentGearMsg.header.stamp    = ros::Time::now();

   mCurrentGearMsg.gear = teleop_control_msgs::Gear::GEAR_UNKNOWN;

   mTeleopGearPublisher.publish(mCurrentGearMsg);
}

GearShifter::~GearShifter()
{
   mCurrentGearMsg.header.stamp = ros::Time::now();
   mCurrentGearMsg.gear         = teleop_control_msgs::Gear::GEAR_UNKNOWN;

   mTeleopGearPublisher.publish(mCurrentGearMsg);
}

void GearShifter::processMsgs()
{
   ros::Rate sleep_timer(gTeleopDeviceMsgPublishRate);

   while (ros::ok())
   {
      ros::spinOnce();

      //      mTeleopGearPublisher.publish(mCurrentGearMsg);

      sleep_timer.sleep();
   }
}

void GearShifter::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   mCurrentJoyMsg = *joy_msg;

   bool is_in_reverse_gear       = readButton(mReverseGearIndex);
   bool is_in_reverse_turbo_gear = readButton(mReverseTurboGearIndex);

   bool is_in_forward_gear       = readButton(mForwardGearIndex);
   bool is_in_forward_turbo_gear = readButton(mForwardTurboGearIndex);

   teleop_control_msgs::Gear gear_msg;
   gear_msg.header = mCurrentGearMsg.header;

   if (is_in_reverse_gear && !is_in_forward_gear && !is_in_forward_turbo_gear
       && !is_in_reverse_turbo_gear)
   {
      gear_msg.gear = teleop_control_msgs::Gear::GEAR_REVERSE;
   }
   else if (is_in_forward_gear && !is_in_reverse_gear && !is_in_forward_turbo_gear
            && !is_in_reverse_turbo_gear)
   {
      gear_msg.gear = teleop_control_msgs::Gear::GEAR_FORWARD;
   }
   else if (is_in_forward_turbo_gear && !is_in_forward_gear && !is_in_reverse_gear
            && !is_in_reverse_turbo_gear)
   {
      gear_msg.gear = teleop_control_msgs::Gear::GEAR_FORWARD_TURBO;
   }
   else if (is_in_reverse_turbo_gear && !is_in_forward_gear && !is_in_reverse_gear
            && !is_in_forward_turbo_gear)
   {
      gear_msg.gear = teleop_control_msgs::Gear::GEAR_REVERSE_TURBO;
   }
   else
   {
      gear_msg.gear = teleop_control_msgs::Gear::GEAR_NEUTRAL;
   }

   ROS_DEBUG_STREAM("joy msg gear: %d" << gear_msg.gear);
   ROS_DEBUG_STREAM("current gear: %d" << mCurrentGearMsg.gear);

   if (mCurrentGearMsg.gear != gear_msg.gear)
   {
      mCurrentGearMsg.gear         = gear_msg.gear;
      mCurrentGearMsg.header.stamp = ros::Time::now();

      mTeleopGearPublisher.publish(mCurrentGearMsg);
   }
}

bool GearShifter::readButton(int device_index)
{
   if (!isButtonIndexValid(device_index))
   {
      return false;
   }

   bool raw_value = mCurrentJoyMsg.buttons.at(device_index);

   return raw_value;
}

bool GearShifter::isAxisIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.axes.size();
}

bool GearShifter::isButtonIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.buttons.size();
}
} // namespace Teleop
