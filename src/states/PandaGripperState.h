#pragma once

#include <mc_control/fsm/State.h>
#include <mc_control/fsm/Controller.h>

#include <devices/Hand.h>

namespace mc_panda
{

/** Stop all commands execution on the related panda robot */
struct PandaGripperState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::string robot_;
  Hand * gripper_;
  double width,speed,force,epsilon_outer,epsilon_inner;
  bool waiting_ = true; // if true do not wait for the command completion
  bool requested_ = false;
  bool done_ = false;
  bool success_=false;

  bool request();
};

} // namespace mc_panda
