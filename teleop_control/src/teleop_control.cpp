#include <teleop_control/teleop_control.h>

#include <iostream>

namespace Teleop
{
const double gTwistPublishRate           = 10.0;
const double gScaleLinearDefault         = 1.0;
const double gScaleAngularDefault        = 1.0;
const double gScaleTurboDefault          = 1.5;
const double gThrottleActiveThreshold    = 0.2;
const double gBrakeActiveThreshold       = 0.2;
const double gDeadmanActiveThreshold     = 0.0;
const double gGearScalingFactorMagnitude = 1.0;

ControlNode::ControlNode(ros::NodeHandle* node_handle, const std::string& node_name)
  : mNodeHandle(*node_handle),
    mCurrentTwistMsg(),
    mCurrentTeleopDeviceMsg(),
    mCurrentGearMsg(),
    mScaleLinear(gScaleLinearDefault),
    mScaleSteering(gScaleAngularDefault),
    mScaleTurbo(gScaleTurboDefault),
    mGearScalingFactor(gGearScalingFactorMagnitude),
    mIsDeadmanRequired(false),
    mIsInTurboMode(false),
    mIsTurboAllowed(false)
{
   std::string node_namespace(node_name + "/");

   mNodeHandle.param(node_namespace + "scale_linear", mScaleLinear, mScaleLinear);
   mNodeHandle.param(node_namespace + "scale_angular", mScaleSteering, mScaleSteering);
   mNodeHandle.param(node_namespace + "scale_turbo", mScaleTurbo, mScaleTurbo);

   mNodeHandle.param(node_namespace + "turbo_allowed", mIsTurboAllowed, mIsTurboAllowed);
   mNodeHandle.param(node_namespace + "deadman_required", mIsDeadmanRequired, mIsDeadmanRequired);

   mTwistPublisher = mNodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   mTeleopDeviceSubscriber =
         mNodeHandle.subscribe("teleop_device", 10, &ControlNode::teleopDeviceMsgCallback, this);

   mGearSubscriber = mNodeHandle.subscribe("gear", 10, &ControlNode::gearMsgCallback, this);

   stopVehicle();
}

ControlNode::~ControlNode()
{
   stopVehicle();
}

void ControlNode::teleopDeviceMsgCallback(
      const teleop_control_msgs::TeleopDevice::ConstPtr& teleop_device_msg)
{
   mCurrentTeleopDeviceMsg = *teleop_device_msg;

   if (!isDeadmanPressed())
   {
      std::cout << __PRETTY_FUNCTION__ << "\tdeadman is not pressed!" << std::endl;
      zeroMotion();
      return;
   }

   processGearButtonStates();

   if (mCurrentTeleopDeviceMsg.brake > gBrakeActiveThreshold)
   {

      std::cout << __PRETTY_FUNCTION__ << "\tbrake is pressed!" << std::endl;

      // deceleration?
      mCurrentTwistMsg.linear.x = std::abs(mCurrentTwistMsg.linear.x) *
                                  (1.0 - mCurrentTeleopDeviceMsg.brake) * mGearScalingFactor;
      return;
   }

   if (mCurrentGearMsg.gear == teleop_control_msgs::Gear::GEAR_NEUTRAL ||
       mCurrentGearMsg.gear == teleop_control_msgs::Gear::GEAR_UNKNOWN)
   {
      std::cout << __PRETTY_FUNCTION__ << "\tin neutral or unknown gear!" << std::endl;

      zeroMotion();
      return;
   }

   if (std::abs(mCurrentTeleopDeviceMsg.throttle) <= gThrottleActiveThreshold)
   {
      std::cout << __PRETTY_FUNCTION__ << "\tthrottle is not pressed!" << std::endl;

      zeroMotion();
      return;
   }

   mCurrentTwistMsg.linear.x = mScaleLinear * mCurrentTeleopDeviceMsg.throttle * mGearScalingFactor;

   if (isTurboModeActive())
   {
      mCurrentTwistMsg.linear.x *= mScaleTurbo;
   }

   mCurrentTwistMsg.angular.z =
         mScaleSteering * mCurrentTeleopDeviceMsg.steering_angle /** mGearScalingFactor*/;

   std::cout << __PRETTY_FUNCTION__ << "\tthrottle is pressed: " << mCurrentTwistMsg.linear.x
             << std::endl;
}

void ControlNode::gearMsgCallback(const teleop_control_msgs::Gear::ConstPtr& gear_msg)
{
   mCurrentGearMsg = *gear_msg;
}

void ControlNode::processMsgs()
{
   ros::Rate sleep_timer(gTwistPublishRate);

   while (ros::ok())
   {
      ros::spinOnce();

      mTwistPublisher.publish(mCurrentTwistMsg);

      sleep_timer.sleep();
   }
}

bool ControlNode::isDeadmanPressed()
{
   if (!mIsDeadmanRequired)
   {
      return true;
   }

   return mCurrentTeleopDeviceMsg.deadman > gDeadmanActiveThreshold;
}

bool ControlNode::isTurboModeActive()
{
   mIsInTurboMode = mCurrentGearMsg.gear == teleop_control_msgs::Gear::GEAR_FORWARD_TURBO ||
                    mCurrentGearMsg.gear == teleop_control_msgs::Gear::GEAR_REVERSE_TURBO;

   return mIsTurboAllowed && mIsInTurboMode;
}

void ControlNode::processGearButtonStates()
{
   double gear_factor = 0.0;

   switch (mCurrentGearMsg.gear)
   {
      case teleop_control_msgs::Gear::GEAR_FORWARD:
      {
         gear_factor = 1.0 * gGearScalingFactorMagnitude;
         break;
      }
      case teleop_control_msgs::Gear::GEAR_FORWARD_TURBO:
      {
         gear_factor = 1.0 * gGearScalingFactorMagnitude;
         break;
      }
      case teleop_control_msgs::Gear::GEAR_REVERSE:
      {
         gear_factor = -1.0 * gGearScalingFactorMagnitude;
         break;
      }
      case teleop_control_msgs::Gear::GEAR_REVERSE_TURBO:
      {
         gear_factor = -1.0 * gGearScalingFactorMagnitude;
         break;
      }
      case teleop_control_msgs::Gear::GEAR_NEUTRAL:
      case teleop_control_msgs::Gear::GEAR_UNKNOWN:
      {
         break;
      }
   }

   mGearScalingFactor = gear_factor;
}

void ControlNode::stopVehicle()
{
   zeroMotion();
   mTwistPublisher.publish(mCurrentTwistMsg);
}

void ControlNode::zeroMotion()
{
   mCurrentTwistMsg.linear.x  = 0.0;
   mCurrentTwistMsg.angular.z = 0.0;
}
} // namespace Teleop
