#include <teleop_control/teleop_flight_stick.h>

namespace Teleop
{
const double gBrakeMaxMagnitude          = 1.0;
const double gDeadmanMaxMagnitude        = 1.0;
const double gTeleopDeviceMsgPublishRate = 10.0;
const int    gThrottleAxisIndexDefault   = 0;
const int    gSteeringAxisIndexDefault   = 1;
const int    gDeadmanButtonIndexDefault  = 0;
const int    gTurboButtonIndexDefault    = 1;

FlightStick::FlightStick(ros::NodeHandle* node_handle, const std::string& node_name)
   : mNodeHandle(*node_handle),
     mCurrentJoyMsg(),
     mCurrentTeleopDeviceMsg(),
     mCurrentGearMsg(),
     mThrottleAxisIndex(gThrottleAxisIndexDefault),
     mSteeringAxisIndex(gSteeringAxisIndexDefault),
     mDeadmanButtonIndex(gDeadmanButtonIndexDefault),
     mTurboButtonIndex(gTurboButtonIndexDefault)
{
   std::string node_namespace(node_name + "/");

   mNodeHandle.param(node_namespace + "axis_throttle", mThrottleAxisIndex, mThrottleAxisIndex);
   mNodeHandle.param(node_namespace + "axis_steering", mSteeringAxisIndex, mSteeringAxisIndex);

   mNodeHandle.param(node_namespace + "button_deadman", mDeadmanButtonIndex, mDeadmanButtonIndex);
   mNodeHandle.param(node_namespace + "button_turbo", mTurboButtonIndex, mTurboButtonIndex);

   mTeleopDevicePublisher =
      mNodeHandle.advertise<teleop_control_msgs::TeleopDevice>("teleop_device", 1);

   mTeleopGearPublisher = mNodeHandle.advertise<teleop_control_msgs::Gear>("gear", 1);

   mJoySubscriber = mNodeHandle.subscribe("joy", 10, &FlightStick::joyMsgCallback, this);

   mCurrentTeleopDeviceMsg.header.frame_id = "";
   brake();

   mCurrentGearMsg.header.frame_id = "";
   setUnknownGear();
}

FlightStick::~FlightStick()
{
   brake();
   setUnknownGear();
}

void FlightStick::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   mCurrentJoyMsg = *joy_msg;

   teleop_control_msgs::TeleopDevice teleop_device_msg;
   teleop_device_msg.header = mCurrentTeleopDeviceMsg.header;

   teleop_control_msgs::Gear gear_msg;
   gear_msg.header = mCurrentGearMsg.header;

   teleop_device_msg.throttle = readThrottle();

   bool in_forward_gear = teleop_device_msg.throttle > 0.0;
   bool in_reverse_gear = teleop_device_msg.throttle < 0.0;

   bool use_turbo = mCurrentJoyMsg.buttons.at(mTurboButtonIndex);

   if (in_forward_gear)
   {
      if (use_turbo)
      {
         gear_msg.gear = teleop_control_msgs::Gear::GEAR_FORWARD_TURBO;
      }
      else
      {
         gear_msg.gear = teleop_control_msgs::Gear::GEAR_FORWARD;
      }
   }
   else if (in_reverse_gear)
   {
      if (use_turbo)
      {
         gear_msg.gear = teleop_control_msgs::Gear::GEAR_REVERSE_TURBO;
      }
      else
      {
         gear_msg.gear = teleop_control_msgs::Gear::GEAR_REVERSE;
      }
   }
   else
   {
      gear_msg.gear = teleop_control_msgs::Gear::GEAR_NEUTRAL;
   }

   if (mCurrentGearMsg != gear_msg)
   {
      mCurrentGearMsg              = gear_msg;
      mCurrentGearMsg.header.stamp = ros::Time::now();

      mTeleopGearPublisher.publish(mCurrentGearMsg);
   }

   if (in_forward_gear || in_reverse_gear)
   {
      teleop_device_msg.brake          = 0.0;
      teleop_device_msg.steering_angle = readSteeringAngle();
      teleop_device_msg.deadman        = readDeadman();

      if (mCurrentTeleopDeviceMsg != teleop_device_msg)
      {
         mCurrentTeleopDeviceMsg              = teleop_device_msg;
         mCurrentTeleopDeviceMsg.header.stamp = ros::Time::now();

         mTeleopDevicePublisher.publish(mCurrentTeleopDeviceMsg);
      }
   }
   else
   {
      brake();
   }
}

void FlightStick::processMsgs()
{
   ros::Rate sleep_timer(gTeleopDeviceMsgPublishRate);

   ros::spin();
}

double FlightStick::readThrottle()
{
   return readHardwareInputDevice(mThrottleAxisIndex);
}

double FlightStick::readSteeringAngle()
{
   return readHardwareInputDevice(mSteeringAxisIndex);
}

double FlightStick::readDeadman()
{
   bool deadman = readButton(mDeadmanButtonIndex);

   if (deadman)
   {
      return gDeadmanMaxMagnitude;
   }

   return 0.0;
}

double FlightStick::readHardwareInputDevice(int device_index)
{
   if (!isAxisIndexValid(device_index))
   {
      return 0.0;
   }

   return mCurrentJoyMsg.axes.at(device_index);
}

bool FlightStick::readButton(int device_index)
{
   if (!isButtonIndexValid(device_index))
   {
      return 0.0;
   }

   return mCurrentJoyMsg.buttons.at(device_index);
}

bool FlightStick::isAxisIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.axes.size();
}

bool FlightStick::isButtonIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.buttons.size();
}

void FlightStick::brake()
{
   mCurrentTeleopDeviceMsg.header.stamp = ros::Time::now();

   mCurrentTeleopDeviceMsg.brake          = gBrakeMaxMagnitude;
   mCurrentTeleopDeviceMsg.throttle       = 0.0;
   mCurrentTeleopDeviceMsg.steering_angle = 0.0;
   mCurrentTeleopDeviceMsg.deadman        = 0.0;

   mTeleopDevicePublisher.publish(mCurrentTeleopDeviceMsg);
}

void FlightStick::setUnknownGear()
{
   mCurrentGearMsg.header.stamp = ros::Time::now();
   mCurrentGearMsg.gear         = teleop_control_msgs::Gear::GEAR_UNKNOWN;

   mTeleopGearPublisher.publish(mCurrentGearMsg);
}
} // namespace Teleop
