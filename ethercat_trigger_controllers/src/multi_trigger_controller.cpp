
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

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include "ethercat_trigger_controllers/multi_trigger_controller.h"
#include "ros/console.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( controller::MultiTriggerController, pr2_controller_interface::Controller)

using std::string;
using namespace controller;

MultiTriggerController::MultiTriggerController()
{
  ROS_DEBUG("creating controller...");
}

MultiTriggerController::~MultiTriggerController()
{
}

void MultiTriggerController::update()
{
  int maxloops = 10; // @todo Workaround to avoid breaking realtime in response to #3274. Need to revamp things to fix this better.
  
  if (!config_.transitions.empty() && config_mutex_.try_lock())
  { // If we missed the lock, then hold current value for now.
    ros::Time cur_time = robot_->getTime();

    while (cur_time.toSec() >= transition_time_) // Usually only happens at most once per update.
    {
      if (!maxloops--)
      {
        //ROS_ERROR("MultiTriggerController exceeded iteration limit. Please report this to Blaise."); // @todo remove this
        break;
      }

      //ROS_INFO("hit %f %f %i", cur_time.toSec(), transition_time_, config_.transitions[transition_index_].value);
      // Do the transition
      digital_out_command_->data_ = config_.transitions[transition_index_].value;
      if (pubs_[transition_index_] && pubs_[transition_index_]->trylock())
      {
        pubs_[transition_index_]->msg_.stamp = cur_time;
        pubs_[transition_index_]->unlockAndPublish();
      }
      
      // Prepare for next transition
      if (++transition_index_ == config_.transitions.size())
      {
        transition_index_ = 0;
        transition_period_++;
      }
      transition_time_ = config_.transitions[transition_index_].time + config_.period * transition_period_ + config_.zero_offset;
    }
    config_mutex_.unlock();
  }
}

template <class T>
static bool parseParamList(ros::NodeHandle nh, std::string param, std::vector<T> &rslt)
{
  std::string param_value;
  nh.getParam(param, param_value);
  std::stringstream parser(param_value);
  T value;
  while (parser >> value)
    rslt.push_back(value);
    
  bool eof = parser.eof();

  if (!eof)
    ROS_ERROR("Error parsing '%s/%s' argument to MultiTriggerController.", nh.getNamespace().c_str(), param.c_str());

  return eof; // Returns true on success
}

bool MultiTriggerController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
  node_handle_ = n;

  ROS_DEBUG("LOADING TRIGGER CONTROLLER NODE");
  //string prefix = config->Attribute("name");
  //ROS_DEBUG_STREAM("the prefix is "<<prefix);

  assert(robot);
  robot_=robot;

  ROS_DEBUG("MultiTriggerController::init starting");

  // Get the digital out name.

  if (!n.getParam("digital_output", digital_output_name_)){
    ROS_ERROR("MultiTriggerController was not given a digital_output parameter.");
    return false;
  }

  pr2_hardware_interface::DigitalOut *digital_out = robot_->model_->hw_->getDigitalOut(digital_output_name_);
  if (!digital_out)
  {
    ROS_ERROR("MultiTriggerController could not find digital output named \"%s\".",
        digital_output_name_.c_str());
    return false;
  }

  digital_out_command_ = &digital_out->command_;

  n.param("period", config_.period, 1.);
  n.param("zero_offset", config_.zero_offset, 0.);

  std::vector<string> topics;
  std::vector<double> times;
  std::vector<uint32_t> values;
  
  waveform_ = node_handle_.advertise<ethercat_trigger_controllers::MultiWaveform>("waveform", 1, true);
  
  if (parseParamList(n, "times", times) && parseParamList(n, "topics", topics) && parseParamList(n, "values", values))
  {
    if (times.size() != topics.size() || times.size() != values.size())
      ROS_ERROR("'topics', 'times' and 'values' parameters must have same length in %s. Ignoring initial settings.", 
          n.getNamespace().c_str());
    else
    {
      for (unsigned int i = 0; i < times.size(); i++)
      {
        ethercat_trigger_controllers::MultiWaveformTransition t;
        t.time = times[i];
        t.value = values[i];
        t.topic = topics[i];
        config_.transitions.push_back(t);
      }

      ethercat_trigger_controllers::SetMultiWaveform::Request req;
      req.waveform = config_;
      ethercat_trigger_controllers::SetMultiWaveform::Response dummy;
      setMultiWaveformSrv(req, dummy);
    }
  }

  set_waveform_handle_ = node_handle_.advertiseService("set_waveform", &MultiTriggerController::setMultiWaveformSrv, this);

  ROS_DEBUG("MultiTriggerController::init completed successfully.");

  return true;
}

bool MultiTriggerController::setMultiWaveformSrv(
    ethercat_trigger_controllers::SetMultiWaveform::Request &req,
    ethercat_trigger_controllers::SetMultiWaveform::Response &resp)
{
  std::vector<boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Header> > > new_pubs;
  new_pubs.reserve(config_.transitions.size());
  ethercat_trigger_controllers::MultiWaveform &new_config = req.waveform;

  double prev_time = -1; // There is a check for negative values below.
  double now = ros::Time::now().toSec();
  double new_transition_period = round((now - new_config.zero_offset) / new_config.period);
  double current_period_start = new_config.zero_offset + new_transition_period * new_config.period;
  double now_offset = now - current_period_start;
  unsigned int new_transition_index = 0;
  resp.success = true;

  if (new_transition_period <= 0)
  {
    resp.status_message = "MultiTrigger period must be >0.";
    resp.success = false;
  }

  for (std::vector<ethercat_trigger_controllers::MultiWaveformTransition>::iterator trans = new_config.transitions.begin();
      trans != new_config.transitions.end() && resp.success; trans++)
  {
    if (trans->time < now_offset)
      new_transition_index++;

    if (trans->time < 0 || trans->time >= new_config.period)
    {
      resp.status_message = (boost::format("MultiTriggerController::setMultiWaveformSrv transition time (%f) must be >= 0 and < period (%f).")%
        trans->time%new_config.period).str();
      resp.success = false;
    }
    
    if (prev_time >= trans->time)
    {
      resp.status_message = (boost::format("MultiTriggerController::setMultiWaveformSrv transition times must be in increasing order. %f >= %f")% 
          prev_time%trans->time).str();
      resp.success = false;
    }
  }

  if (new_transition_index == new_config.transitions.size())
  {
    new_transition_index = 0;
    new_transition_period++;
  }

  double new_transition_time = current_period_start + new_config.transitions[new_transition_index].time;
  
//  ROS_DEBUG("MultiTriggerController::setMultiWaveformSrv completed successfully"
//      " rr=%f ph=%f al=%i r=%i p=%i dc=%f.", config_.rep_rate, config_.phase,
//      config_.active_low, config_.running, config_.pulsed, config_.duty_cycle);

  if (resp.success)
  { 
    for (std::vector<ethercat_trigger_controllers::MultiWaveformTransition>::iterator trans = new_config.transitions.begin();
        trans != new_config.transitions.end() && resp.success; trans++)
    {
      boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Header> > new_pub;
        
      if (trans->topic.compare("-"))
        new_pub.reset(new realtime_tools::RealtimePublisher<std_msgs::Header>(node_handle_, trans->topic, 10));

      new_pubs.push_back(new_pub);
    }

    boost::mutex::scoped_lock lock(config_mutex_);
    config_ = new_config;
    pubs_ = new_pubs;
    transition_period_ = new_transition_period;
    transition_index_ = new_transition_index;
    transition_time_ = new_transition_time;
    waveform_.publish(req.waveform);
  }
  else
    ROS_ERROR("%s", resp.status_message.c_str());
  
  return true;
}
