#include <teleop_control/teleop_driving_wheel.h>

namespace Teleop
{
const double gHIDMaxMagnitude              = 1.0;
const double gHIDValueRange                = 2.0 * gHIDMaxMagnitude;
const double gTeleopDeviceMsgPublishRate   = 10.0;
const int    gSteeringAxisIndexDefault     = 0;
const int    gDeadmanPedalAxisIndexDefault = 1;
const int    gThrottleAxisIndexDefault     = 2;
const int    gBrakeAxisIndexDefault        = 3;

DrivingWheel::DrivingWheel(ros::NodeHandle* node_handle, const std::string& node_name)
   : mNodeHandle(*node_handle),
     mCurrentJoyMsg(),
     mCurrentTeleopDeviceMsg(),
     mThrottleAxisIndex(gThrottleAxisIndexDefault),
     mBrakeAxisIndex(gBrakeAxisIndexDefault),
     mSteeringAxisIndex(gSteeringAxisIndexDefault),
     mDeadmanPedalAxisIndex(gDeadmanPedalAxisIndexDefault)
{
   std::string node_namespace(node_name + "/");

   mNodeHandle.param(node_namespace + "axis_throttle", mThrottleAxisIndex, mThrottleAxisIndex);
   mNodeHandle.param(node_namespace + "axis_steering", mSteeringAxisIndex, mSteeringAxisIndex);
   mNodeHandle.param(node_namespace + "axis_brake_pedal", mBrakeAxisIndex, mBrakeAxisIndex);

   mNodeHandle.param(node_namespace + "axis_deadman",
                     mDeadmanPedalAxisIndex,
                     mDeadmanPedalAxisIndex);

   mTeleopDevicePublisher =
      mNodeHandle.advertise<teleop_control_msgs::TeleopDevice>("teleop_device", 1);

   mJoySubscriber = mNodeHandle.subscribe("joy", 10, &DrivingWheel::joyMsgCallback, this);

   mCurrentTeleopDeviceMsg.header.frame_id = "";
   brake();
}

DrivingWheel::~DrivingWheel()
{
   brake();
}

void DrivingWheel::joyMsgCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   mCurrentJoyMsg = *joy_msg;

   teleop_control_msgs::TeleopDevice teleop_device_msg;
   teleop_device_msg.header = mCurrentTeleopDeviceMsg.header;

   teleop_device_msg.brake          = readBrake();
   teleop_device_msg.throttle       = readThrottle();
   teleop_device_msg.steering_angle = readSteeringAngle();
   teleop_device_msg.deadman        = readDeadman();

   if (mCurrentTeleopDeviceMsg != teleop_device_msg)
   {
      mCurrentTeleopDeviceMsg              = teleop_device_msg;
      mCurrentTeleopDeviceMsg.header.stamp = ros::Time::now();

      mTeleopDevicePublisher.publish(mCurrentTeleopDeviceMsg);
   }
}

void DrivingWheel::processMsgs()
{
   ros::Rate sleep_timer(gTeleopDeviceMsgPublishRate);

   ros::spin();
}

double DrivingWheel::readThrottle()
{
   return readHardwareInputDevice(mThrottleAxisIndex);
}

double DrivingWheel::readBrake()
{
   return readHardwareInputDevice(mBrakeAxisIndex);
}

double DrivingWheel::readSteeringAngle()
{
   if (!isAxisIndexValid(mSteeringAxisIndex))
   {
      return 0.0;
   }

   return mCurrentJoyMsg.axes.at(mSteeringAxisIndex);
}

double DrivingWheel::readDeadman()
{
   return readHardwareInputDevice(mDeadmanPedalAxisIndex);
}

double DrivingWheel::readHardwareInputDevice(int device_index)
{
   if (!isAxisIndexValid(device_index))
   {
      return 0.0;
   }

   double raw_value      = mCurrentJoyMsg.axes.at(device_index);
   double adjusted_value = (raw_value + gHIDMaxMagnitude) / gHIDValueRange;

   return adjusted_value;
}

bool DrivingWheel::isAxisIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.axes.size();
}

bool DrivingWheel::isButtonIndexValid(int index)
{
   return static_cast<size_t>(index) < mCurrentJoyMsg.buttons.size();
}

void DrivingWheel::brake()
{
   mCurrentTeleopDeviceMsg.header.stamp = ros::Time::now();

   mCurrentTeleopDeviceMsg.brake          = 100.0;
   mCurrentTeleopDeviceMsg.throttle       = 0.0;
   mCurrentTeleopDeviceMsg.steering_angle = 0.0;
   mCurrentTeleopDeviceMsg.deadman        = 0.0;

   mTeleopDevicePublisher.publish(mCurrentTeleopDeviceMsg);
}
} // namespace Teleop
