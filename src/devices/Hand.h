#pragma once

#include <mc_panda/devices/api.h>

#include <mc_rbdyn/Device.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/log/Logger.h>

#include <franka/gripper.h>
#include <franka/gripper_state.h>

#include <chrono>
#include <thread>
#include <vector>

namespace mc_panda
{

/** This device is an asynchronous wrapper around the synchronous Gripper
 * interface provided by libfranka */
struct MC_PANDA_DEVICES_DLLAPI Hand : public mc_rbdyn::Device
{
  static constexpr auto name = "Hand";
  using StatusInt = std::underlying_type<franka::GripperState>;



  /** Get the hand associated to the provided robot
   *
   * \returns nullptr if the device does not exist in this robot
   */
  static Hand * get(mc_rbdyn::Robot & robot);

  /** Constructor
   *
   * @param name Name of the hand
   *
   */
  Hand(const std::string & parent, const sva::PTransformd & X_p_d);

  ~Hand() override;

  /** Connect the hand device to an actual hand, the hand operations are then done in a background thread*/
  bool connect(const std::string & ip);

  /** Disconnect from the actual hand */
  void disconnect();

  /** Access the gripper state */
  const franka::GripperState & state() const;



  /** True if the hand is currently busy */
  bool busy() const;

  /** True if the last command succeeded, false otherwise */
  bool success() const;

  /** Message describing the latest error */
  const std::string & error() const;
  
  /**
   * Grasp as an object.
   *
   * @param[in] width Size of the object to grasp. [m]
   * @param[in] speed Closing speed. [m/s]
   * @param[in] force Grasping force. [N]
   * @param[in] epsilon_inner Maximum tolerated deviation when the actual grasped width is smaller than the commanded grasp width.
   * @param[in] epsilon_outer	 Maximum tolerated deviation when the actual grasped width is larger than the commanded grasp width.
   * 
   * @return True if the command was requested, false otherwise
   */
  bool grasp (double   width,
               double 	speed,
               double 	force,
               double 	epsilon_inner = 0.005,
               double 	epsilon_outer = 0.005);

  /**
   * Move as an object.
   *
   * @param[in] width Size of the object to grasp. [m]
   * @param[in] speed Closing speed. [m/s]

   * @return True if the command was requested, false otherwise
   */
  bool move (double   width,
               double 	speed);

  /**
   * homing as an object.
   *
   * @return True if the command was requested, false otherwise
   */
  bool homing ();

  /**
   * Stops a currently running  gripper.
   *
   * @return True if the command was requested, false otherwise
   */
  bool stop();
  /**
   * Returns the current width value
   *@return Double width value measured in [m]
  */
  double WidthValue();
  void addToLogger(mc_rtc::Logger & logger, const std::string & prefix);

  void removeFromLogger(mc_rtc::Logger & logger, const std::string & prefix);

  mc_rbdyn::DevicePtr clone() const override;

private:
  // Status of the hand

  // Only non-null if the hand is connected
  std::unique_ptr<franka::Gripper> gripper_;
  // Thread for reading the gripper state
  std::thread stateThread_;
  // Mutex for protecting the gripper state
  mutable std::mutex stateMutex_;
  // Current state
  franka::GripperState state_;
  // Thread for sending commands
  std::thread commandThread_;
  // Thread for interrupting commands
  std::thread interruptThread_;
  // Only true while the gripper is connected
  std::atomic<bool> connected_{false};
  // Only true while a command is being executed
  std::atomic<bool> busy_{false};
  // Only true if a command has been interrupted
  std::atomic<bool> interrupted_{false};
  struct Command
  {
    std::string name;
    std::function<bool()> callback;
  };
  // Only valid while a command is being executed
  Command command_;
  // Represent the last command executed
  uint8_t last_command_id_ = 0;
  // Store the last command success
  bool success_ = false;
  // Store the last command error (if any)
  std::string error_ = "";
};

typedef std::vector<Hand, Eigen::aligned_allocator<Hand>> HandVector;

} // namespace mc_panda
namespace mc_rtc
{



} // namespace mc_rtc
