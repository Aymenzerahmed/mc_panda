#include "Hand.h"

#include <mc_rbdyn/Robot.h>

#include <franka/exception.h>

namespace mc_panda
{

Hand::Hand(const std::string & parent, const sva::PTransformd & X_p_d) : mc_rbdyn::Device(Hand::name, parent, X_p_d)
{
  type_ = "Hand";
}

Hand::~Hand()
{
  disconnect();
}

Hand * Hand::get(mc_rbdyn::Robot & robot)
{
  if(robot.hasDevice<Hand>(Hand::name))
  {
    return &(robot.device<Hand>(Hand::name));
  }
  return nullptr;
}

mc_rbdyn::DevicePtr Hand::clone() const
{
  if(gripper_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot copy a connected Hand");
  }
  auto ret = new Hand(parent_, X_p_s_);
  ret->state_ = this->state_;
  return mc_rbdyn::DevicePtr(ret);
}

void Hand::addToLogger(mc_rtc::Logger & logger, const std::string & prefix)
{
  
  logger.addLogEntry(prefix + "_temperature", [this]() { return state_.temperature; });
  logger.addLogEntry(prefix + "_width", [this]() { return state_.width; });
  logger.addLogEntry(prefix + "_isgrasped", [this]() { return state_.is_grasped; });
}

void Hand::removeFromLogger(mc_rtc::Logger & logger, const std::string & prefix)
{
  logger.removeLogEntry(prefix + "_temperature");
  logger.removeLogEntry(prefix + "_width");
  logger.removeLogEntry(prefix + "_isgrasped");

}

bool Hand::grasp(double width,double speed,double force,double epsilon_inner ,double 	epsilon_outer)
{
  if(!gripper_)
  {
    return true;
  }
  if(busy_)
  {
    mc_rtc::log::error("{} is already busy executing {} command", name_, command_.name);
    return false;
  }
  busy_ = true;
  
  command_ = {"grasp", [=]() { return gripper_->grasp(width, speed, force, epsilon_inner,epsilon_outer); }};
  last_command_id_ = 1;
  return true;
}
bool Hand::move(double width,double speed)
{
  if(!gripper_)
  {
    return true;
  }
  if(busy_)
  {
    mc_rtc::log::error("{} is already busy executing {} command", name_, command_.name);
    return false;
  }
  busy_ = true;
  
  command_ = {"move", [=]() { return gripper_->move(width, speed); }};
  last_command_id_ = 2;
  return true;
}
bool Hand::homing()
{

  if(!gripper_)
  {
     mc_rtc::log::info("Starting Homing");
    return true;
  }
  if(busy_)
  {
    mc_rtc::log::error("{} is already busy executing {} command", name_, command_.name);
    return false;
  }
  busy_ = true;
 
  command_ = {"homing", [=]() { return gripper_->homing(); }};
  last_command_id_ = 3;
  return true;
}
bool Hand::connect(const std::string & ip)
{
  if(gripper_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("{} is already connected", name_);
  }
  try
  {
    gripper_.reset(new franka::Gripper(ip));
    mc_rtc::log::info("{} connected to {}", name_, ip);
  }
  catch(const franka::NetworkException & exc)
  {
    mc_rtc::log::error("{} failed to connect to {}: {}", name_, ip, exc.what());
    return false;
  }
  catch(const franka::IncompatibleVersionException & exc)
  {
    mc_rtc::log::error("{} imcompatible version with {}: {}", name_, ip, exc.what());
    return false;
  }
  connected_ = true;
  stateThread_ = std::thread([this]() {
    while(connected_)
    {
      try
      {
        auto stateIn = gripper_->readOnce();
        std::unique_lock<std::mutex> lock(stateMutex_);
        state_ = stateIn;
      }
      catch(const franka::NetworkException & exc)
      {
        mc_rtc::log::error("{} connection lost, failed to read state: {}", name_, exc.what());
      }
    }
  });
  commandThread_ = std::thread([this]() {
    while(connected_)
    {
      if(busy_)
      {
        bool s = false;
        std::string error;
        try
        {
          s = command_.callback();
          error = "";
        }
        catch(const franka::CommandException & exc)
        {
          error = exc.what();
          mc_rtc::log::error("{} {} command failed: {}", name_, command_.name, error);
        }
        catch(const franka::NetworkException & exc)
        {
          error = exc.what();
          mc_rtc::log::error("{} connection lost, failed to execute {} command: {}", name_, command_.name, error);
        }
        if(!interrupted_)
        {
          success_ = s;
          error_ = error;
        }
        else
        {
          interrupted_ = false;
        }
        busy_ = false;
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });
  interruptThread_ = std::thread([this]() {
    while(connected_)
    {
      if(interrupted_)
      {
        bool busy = busy_;
        bool s = false;
        std::string error;
        try
        {
          s = gripper_->stop();
          error = "";
        }
        catch(const franka::CommandException & exc)
        {
          error = exc.what();
          mc_rtc::log::error("{} stop command failed: {}", name_, error);
        }
        catch(const franka::NetworkException & exc)
        {
          error = exc.what();
          mc_rtc::log::error("{} connection lost, failed to execute stop command: {}", name_, error);
        }
        success_ = s;
        error_ = error;
        if(!busy)
        {
          interrupted_ = false;
        }
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });
  return true;
}

void Hand::disconnect()
{
  if(!gripper_)
  {
    return;
  }
  connected_ = false;
  commandThread_.join();
  stateThread_.join();
  interruptThread_.join();
  gripper_.reset(nullptr);
}

const franka::GripperState & Hand::state() const
{
  std::unique_lock<std::mutex> lock(stateMutex_);
  return state_;
}



bool Hand::busy() const
{
  return busy_;
}

bool Hand::success() const
{
  return success_;
}

const std::string & Hand::error() const
{
  return error_;
}



bool Hand::stop()
{
  if(!gripper_)
  {
    return true;
  }
  if(interrupted_)
  {
    mc_rtc::log::error("{} stop command has already been requested", name_);
    return false;
  }
  gripper_->stop();
  interrupted_ = true;
  last_command_id_ = 4;
  return true;
}
double Hand::WidthValue()
{
    if(!gripper_)
  {
   state_= gripper_->readOnce();
  }
  else
  {
    /**/
  }
 return state_.width;
}
} // namespace mc_panda
