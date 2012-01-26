
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
#ifndef TRIGGER_CONTROLLER_H
#define TRIGGER_CONTROLLER_H

#include <ros/node_handle.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/robot.h>
#include <ethercat_trigger_controllers/SetWaveform.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Header.h>
#include <boost/scoped_ptr.hpp>

/** @class TriggerController
  * @brief Allows periodic triggering of cameras through the digital output
  * pin of the motor controller boards.
  *
  */

namespace controller
{
typedef ethercat_trigger_controllers::SetWaveform::Request trigger_configuration;

class TriggerControllerNode;

class TriggerController : public pr2_controller_interface::Controller
{
public:
  TriggerController();

  ~TriggerController();

  void update();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  /**
   * \brief Convert time to an unrolled phase (in cycles).
   *
   * At time 0, the unrolled phase is equal to -config.phase. Thereafter,
   * phase increases at a rate of 1/config.rep_rate.
   *
   * \param t Time for which the phase should be evaluated.
   *
   * \param config Trigger configuration for which the phase should be evaluated.
   *
   * \return Unrolled phase in cycles.
   */

  static double getTick(double t, trigger_configuration const &config)
  {
    return  t * config.rep_rate - config.phase;
  }

  /**
   * \brief Convert time to an unrolled phase (in cycles).
   *
   * At time 0, the unrolled phase is equal to -config.phase. Thereafter,
   * phase increases at a rate of 1/config.rep_rate. This function returns
   * the unrolled phase.
   *
   * \param t Time for which the phase should be evaluated.
   *
   * \param config Trigger configuration for which the phase should be evaluated.
   *
   * \return Unrolled phase in cycles.
   */

  static double getTick(const ros::Time &t, trigger_configuration const &config)
  {
    return getTick(t.toSec(), config);
  }

  /**
   * \brief Gets the ros::Time at which a cycle starts.
   *
   * This function takes an unrolled phase, and returns the time at which
   * the current cycle started. That is, the most recent time at which the
   * phase was integer.
   *
   * \param tick Unrolled phase.
   *
   * \param config Trigger configuration for which the cycle start should be evaluated.
   *
   * \return Cycle start time.
   */

  static double getTickStartTimeSecFromPhase(double tick, trigger_configuration const &config)
  {
    return (floor(tick) + config.phase) / config.rep_rate;
  }

  /**
   * \brief Gets the ros::Time at which a cycle starts.
   *
   * This function takes an unrolled phase, and returns the time at which
   * the current cycle started. That is, the most recent time at which the
   * phase was integer.
   *
   * \param tick Unrolled phase.
   *
   * \param config Trigger configuration for which the cycle start should be evaluated.
   *
   * \return Cycle start time.
   */

  static ros::Time getTickStartTimeFromPhase(double tick, trigger_configuration const &config)
  {
    return ros::Time(getTickStartTimeSecFromPhase(tick, config));
  }

  /**
   * \brief Gets the ros::Time at which a cycle starts.
   *
   * This function takes a time, and returns the time at which
   * the current cycle started. That is, the most recent time at which the
   * phase was integer.
   *
   * \param tick Time for which to perform the computation.
   *
   * \param config Trigger configuration for which the cycle start should be evaluated.
   *
   * \return Cycle start time.
   */

  static ros::Time getTickStartTime(ros::Time time, trigger_configuration const &config)
  {
    return ros::Time(getTickStartTimeSec(time.toSec(), config));
  }

  /**
   * \brief Gets the ros::Time at which a cycle starts.
   *
   * This function takes a time, and returns the time at which
   * the current cycle started. That is, the most recent time at which the
   * phase was integer.
   *
   * \param tick Time for which to perform the computation.
   *
   * \param config Trigger configuration for which the cycle start should be evaluated.
   *
   * \return Cycle start time.
   */

  static double getTickStartTimeSec(double time, trigger_configuration const &config)
  {
    return getTickStartTimeSecFromPhase(getTick(time, config), config);
  }

  /**
   * \brief Gets the time during which the output will be active during
   * each cycle.
   *
   * This function determines how much time the output is active for during
   * each cycle.
   *
   * \param config Trigger configuration for which the cycle start should be evaluated.
   *
   * \return Cycle start time.
   */

  static ros::Duration getTickDuration(trigger_configuration &config)
  {
    return ros::Duration(getTickDurationSec(config));
  }

  /**
   * \brief Gets the time during which the output will be active during
   * each cycle.
   *
   * This function determines how much time the output is active for during
   * each cycle.
   *
   * \param config Trigger configuration for which the cycle start should be evaluated.
   *
   * \return Cycle start time.
   */

  static double getTickDurationSec(trigger_configuration &config)
  {
    if (!config.running)
      return 0;
    else if (config.pulsed)
      return 1e-3; // @todo the update rate should be in an include file somewhere.
    else
      return config.duty_cycle / config.rep_rate;
  }

private:
  double getTick();

  bool setWaveformSrv(trigger_configuration &req,
      ethercat_trigger_controllers::SetWaveform::Response &resp);

  pr2_mechanism_model::RobotState * robot_;
  pr2_hardware_interface::DigitalOutCommand *digital_out_command_;

  double prev_tick_;

  ros::ServiceServer set_waveform_handle_;
  ros::NodeHandle node_handle_;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      std_msgs::Header> > rising_edge_pub_, falling_edge_pub_;
  bool last_out_;

  // Configuration of controller.
  trigger_configuration config_;
  std::string actuator_name_;
};

};

#endif

