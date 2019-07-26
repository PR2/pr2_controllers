
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

#include <vector>

#include <ros/node_handle.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/robot.h>
#include <ethercat_trigger_controllers/SetMultiWaveform.h>
#include <ethercat_trigger_controllers/MultiWaveform.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Header.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

/** @class MultiMultiTriggerController
  * @brief Allows complex periodic triggering waveforms through a digital output
  * pin of the motor controller boards.
  *
  */

namespace controller
{
class MultiTriggerController : public pr2_controller_interface::Controller
{
typedef ethercat_trigger_controllers::MultiWaveform config_t;
public:
  MultiTriggerController();

  ~MultiTriggerController();

  void update();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

private:
  boost::mutex config_mutex_; // Held while config_ is changing.

  bool setMultiWaveformSrv(
      ethercat_trigger_controllers::SetMultiWaveform::Request &req,
      ethercat_trigger_controllers::SetMultiWaveform::Response &resp);

  pr2_mechanism_model::RobotState *robot_;
  pr2_hardware_interface::DigitalOutCommand *digital_out_command_;

  // Information about the next transition
  double transition_time_;
  double transition_period_;
  unsigned int transition_index_;

  ros::ServiceServer set_waveform_handle_;
  ros::NodeHandle node_handle_;
  ros::Publisher waveform_;

  std::vector<boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Header> > > pubs_;

  // Configuration of controller.
  config_t config_;
  std::string digital_output_name_;
};

};

#endif

