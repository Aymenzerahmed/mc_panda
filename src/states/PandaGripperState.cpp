#include "PandaGripperState.h"

#include <mc_control/fsm/Controller.h>

#include <devices/Robot.h>


namespace mc_panda
{

void PandaGripperState::configure(const mc_rtc::Configuration & config)
{
  config("robot", robot_);
  config("width", width );// maximum of 0.08~m 
  config("force", force );//the grasping force in doc: 140~N as maximum force and 70~N as continuous force
  config("speed", speed );// traveling speed 0.05~m/s per finger
  config("speed",epsilon_outer); // default value 0.005~m
  config("speed",epsilon_inner); // default value 0.005~m
}


void PandaGripperState::start(mc_control::fsm::Controller & ctl_)
{

  auto & robot = robot_.empty() ? ctl_.robot() : ctl_.robot(robot_);
  gripper_= Hand::get(robot);
    if(!gripper_)
  {
    mc_rtc::log::warning("[{}] State started with robot {} which does not have an mc_panda::Hand device", name(),
                         robot_);
    output("NoHand");
    return;
  }
  //gripper_ -> connect()
  //gripper_-> homing();
  request();
}

bool PandaGripperState::run(mc_control::fsm::Controller & ctl_)
{
  //  mc_rtc::log::info("[{}] Hand gripper requested success: {}",name(), success_);
 if(!gripper_ || (!waiting_ && requested_) || done_)
  {
    return true;
  }

  // Pump still busy from a previous command
  if(!requested_)
  {
    return request();
  }
  // Pump busy from this command
  if(gripper_->busy())
  {
    return false;
  }

  done_ = true;
  if(gripper_->success())
  {
    output("OK");
  }
  else
  {
    gripper_->stop();
    output("GripperFailure!!");
  }
  return true;
}

void PandaGripperState::teardown(mc_control::fsm::Controller & ctl_) {}
bool PandaGripperState::request()
{
  if(!gripper_->busy())
  {
    requested_= gripper_->grasp(width, speed, force, epsilon_inner, epsilon_outer);
    if (requested_)
    {
      mc_rtc::log::info("[{}] Hand gripper requested with width: {}m, speed: {}m/s", name(), width,
                        speed);
      if(!waiting_)
      {
        output("OK");
      }
      success_= gripper_ -> success();
      
    }
  }
  return requested_ && !waiting_;
}
} // namespace mc_panda

EXPORT_SINGLE_STATE("PandaGripper", mc_panda::PandaGripperState)
