
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** Example .xml configuration file
<controllers>
<controller name="cam_controller" type="TriggerControllerNode">
  <actuator name="fl_caster_l_wheel_motor" />
  <waveform rep_rate="10" active_low="0" phase="0" duty_cycle=".5" running="1" pulsed="1" />
</controller>
<controller name="led_controller" type="TriggerControllerNode">
  <actuator name="bl_caster_l_wheel_motor" />
  <waveform rep_rate="10" active_low="0" phase="0" duty_cycle=".5" running="1" pulsed="1" />
</controller>
</controllers>

There are three operating modes:
Constant: (running = 0)
  active_low sets the constant output value
Pulsed: (running = 1, pulsed = 1)
  active_low sets resting output value
  With rate rep_rate (Hz), the output is inverted for one cycle. The pulse is delayed using
  phase varying from 0 to 1.
Rectangular: (running = 1, pulsed = 0)
  Rectangular wave with duty cycle set by duty_cycle (0 to 1), rate rep_rate (Hz), goes to
  !active_low at instant determined by phase.
*/

#include "ethercat_trigger_controllers/trigger_controller.h"
#include "ros/console.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( controller::TriggerController, pr2_controller_interface::Controller)

using std::string;
using namespace controller;

TriggerController::TriggerController()
{
  ROS_DEBUG("creating controller...");
}

TriggerController::~TriggerController()
{
}

double TriggerController::getTick()
{
  return getTick(robot_->getTime(), config_);
}

void TriggerController::update()
{
  ros::Time curtime = robot_->getTime();
  double tick = getTick(curtime, config_);
  bool active = false;

  if (config_.running)
  {
    if (config_.pulsed)
    {
      active = (floor(prev_tick_) != floor(tick));
      //if (active)
      //  ROS_INFO("Triggered (%s)", actuator_name_.c_str()); // KILLME
    }
    else
    {
      active = fmod(tick, 1) < config_.duty_cycle;
      //if (active != fmod(prev_tick_, 1) < config_.duty_cycle)
      //  ROS_INFO("Changed to: %i (%s)", active, actuator_name_.c_str()); // KILLME
    }
  }

  //if (actuator_command_->digital_out_ && !(active ^ config_.active_low))
  //    ROS_DEBUG("digital out falling at time %f", robot_->getTime());

  if (digital_out_command_->data_ != last_out_)
  {
    //ROS_WARN("Contention on digital output %s. Is %i, expected %i.", actuator_name_.c_str(), digital_out_command_->data_, last_out_);
  }

  digital_out_command_->data_ = active ^ config_.active_low;

  if (last_out_ && !digital_out_command_->data_)
  {
    if (falling_edge_pub_ && falling_edge_pub_->trylock())
    {
      falling_edge_pub_->msg_.stamp = curtime;
      falling_edge_pub_->unlockAndPublish();
      //ROS_DEBUG("Published rising edge from %s", actuator_name_.c_str());
    }
    else
    {
      //ROS_WARN("Unable to publish on falling edge of TriggerController %s.", actuator_name_.c_str());
    }
  }
  else if (!last_out_ && digital_out_command_->data_)
  {
    if (rising_edge_pub_ && rising_edge_pub_->trylock())
    {
      rising_edge_pub_->msg_.stamp = curtime;
      rising_edge_pub_->unlockAndPublish();
      //ROS_DEBUG("Published falling edge from %s", actuator_name_.c_str());
    }
    else
    {
      //ROS_WARN("Unable to publish on rising edge of TriggerController %s.", actuator_name_.c_str());
    }
  }
  
  //  ROS_INFO("digital out: %i (%s) %i", actuator_command_->digital_out_, actuator_name_.c_str(), last_out_);

  last_out_ = digital_out_command_->data_;

  prev_tick_ = tick;
}

bool TriggerController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
  node_handle_ = n;

  ROS_DEBUG("LOADING TRIGGER CONTROLLER NODE");
  //string prefix = config->Attribute("name");
  //ROS_DEBUG_STREAM("the prefix is "<<prefix);

  assert(robot);
  robot_=robot;

  ROS_DEBUG("TriggerController::init starting");

  // Get the actuator name.

  if (!n.getParam("actuator", actuator_name_)){
    ROS_ERROR("TriggerController was not given an actuator.");
    return false;
  }

  pr2_hardware_interface::DigitalOut *digital_out = robot_->model_->hw_->getDigitalOut(actuator_name_);
  if (!digital_out)
  {
    ROS_ERROR("TriggerController could not find digital out named \"%s\".",
        actuator_name_.c_str());
    return false;
  }

  digital_out_command_ = &digital_out->command_;
  digital_out_command_->data_ = false;
  last_out_ = false;

  // Get the startup configuration (pulsed or constant)

#define bparam(name, var, val) \
  {\
    bool tmp;\
    n.param(name, tmp, val);\
    var = tmp;\
  }
  n.param("rep_rate", config_.rep_rate, 1.);
  n.param("phase", config_.phase, 0.);
  n.param("duty_cycle", config_.duty_cycle, .5);
  bparam("active_low", config_.active_low, false);
  bparam("running", config_.running, false);
  bparam("pulsed", config_.pulsed, true);
#undef bparam

  prev_tick_ = getTick();

  set_waveform_handle_ = node_handle_.advertiseService("set_waveform", &TriggerController::setWaveformSrv, this);

  rising_edge_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Header>(n, "rising_edge_timestamps", 10));
  falling_edge_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Header>(n, "falling_edge_timestamps", 10));

  ROS_DEBUG("TriggerController::init completed successfully"
      " rr=%f ph=%f al=%i r=%i p=%i dc=%f.",
      config_.rep_rate, config_.phase, config_.active_low, config_.running, config_.pulsed, config_.duty_cycle);

  return true;
}

bool TriggerController::setWaveformSrv(
    trigger_configuration &req,
    ethercat_trigger_controllers::SetWaveform::Response &resp)
{
  // FIXME This should be safe despite the asynchronous barrier. Should I
  // be doing anything special to ensure that things get written in order?
  config_.running = false; // Turn off pulsing before we start.
  config_.rep_rate = req.rep_rate;
  config_.phase = req.phase;
  config_.duty_cycle = req.duty_cycle;
  config_.active_low = !!req.active_low;
  config_.pulsed = !!req.pulsed;
  config_.running = !!req.running;

  ROS_DEBUG("TriggerController::setWaveformSrv completed successfully"
      " rr=%f ph=%f al=%i r=%i p=%i dc=%f.", config_.rep_rate, config_.phase,
      config_.active_low, config_.running, config_.pulsed, config_.duty_cycle);

  return true;
}
