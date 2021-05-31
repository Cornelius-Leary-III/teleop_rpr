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

   mCurrentTwistMsg.linear.x  = 0.0;
   mCurrentTwistMsg.angular.z = 0.0;
   mTwistPublisher.publish(mCurrentTwistMsg);
}

ControlNode::~ControlNode()
{
   mCurrentTwistMsg.linear.x  = 0.0;
   mCurrentTwistMsg.angular.z = 0.0;
   mTwistPublisher.publish(mCurrentTwistMsg);
}

void ControlNode::teleopDeviceMsgCallback(
   const teleop_control_msgs::TeleopDevice::ConstPtr& teleop_device_msg)
{
   mCurrentTeleopDeviceMsg = *teleop_device_msg;

   if (!isDeadmanPressed())
   {
      mCurrentTwistMsg.linear.x  = 0.0;
      mCurrentTwistMsg.angular.z = 0.0;
      return;
   }

   if (mCurrentTeleopDeviceMsg.brake > gBrakeActiveThreshold)
   {
      processGearButtonStates();

      // deceleration?
      mCurrentTwistMsg.linear.x = std::abs(mCurrentTwistMsg.linear.x)
                                  * (1.0 - mCurrentTeleopDeviceMsg.brake) * mGearScalingFactor;
      return;
   }

   if (mCurrentGearMsg.gear == teleop_control_msgs::Gear::GEAR_NEUTRAL
       || mCurrentGearMsg.gear == teleop_control_msgs::Gear::GEAR_UNKNOWN)
   {
      mCurrentTwistMsg.linear.x  = 0.0;
      mCurrentTwistMsg.angular.z = 0.0;
      return;
   }

   if (mCurrentTeleopDeviceMsg.throttle <= gThrottleActiveThreshold)
   {
      mCurrentTwistMsg.linear.x  = 0.0;
      mCurrentTwistMsg.angular.z = 0.0;
      return;
   }

   mCurrentTwistMsg.linear.x = mScaleLinear * mCurrentTeleopDeviceMsg.throttle * mGearScalingFactor;

   if (isTurboModeActive())
   {
      mCurrentTwistMsg.linear.x *= mScaleTurbo;
   }

   mCurrentTwistMsg.angular.z =
      mScaleSteering * mCurrentTeleopDeviceMsg.steering_angle /** mGearScalingFactor*/;
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
   mIsInTurboMode = mCurrentGearMsg.gear == teleop_control_msgs::Gear::GEAR_FORWARD_TURBO
                    || mCurrentGearMsg.gear == teleop_control_msgs::Gear::GEAR_REVERSE_TURBO;

   return mIsTurboAllowed && mIsInTurboMode;
}

void ControlNode::processGearButtonStates()
{
   switch (mCurrentGearMsg.gear)
   {
      case teleop_control_msgs::Gear::GEAR_FORWARD:
      {
         mGearScalingFactor = 1.0 * gGearScalingFactorMagnitude;
         break;
      }
      case teleop_control_msgs::Gear::GEAR_FORWARD_TURBO:
      {
         mGearScalingFactor = 1.0 * gGearScalingFactorMagnitude;
         break;
      }
      case teleop_control_msgs::Gear::GEAR_REVERSE:
      {
         mGearScalingFactor = -1.0 * gGearScalingFactorMagnitude;
         break;
      }
      case teleop_control_msgs::Gear::GEAR_REVERSE_TURBO:
      {
         mGearScalingFactor = -1.0 * gGearScalingFactorMagnitude;
         break;
      }
      case teleop_control_msgs::Gear::GEAR_NEUTRAL:
      case teleop_control_msgs::Gear::GEAR_UNKNOWN:
      {
         mGearScalingFactor = 0.0;
         break;
      }
   }
}
} // namespace Teleop
