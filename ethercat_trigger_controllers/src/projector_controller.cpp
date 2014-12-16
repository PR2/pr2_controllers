
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
#include "ethercat_trigger_controllers/projector_controller.h"
#include "ros/console.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( controller::ProjectorController, pr2_controller_interface::Controller)

using std::string;
using namespace controller;

ProjectorController::ProjectorController()
{
  ROS_DEBUG("creating controller...");
}

ProjectorController::~ProjectorController()
{
}

void ProjectorController::update()
{
  /// @todo These calculations stink but they will do for now...
  uint32_t rising = projector_->state_.rising_timestamp_us_;
  uint32_t falling = projector_->state_.falling_timestamp_us_;
  double curtime = robot_->getTime().toSec();
  double delta = curtime - start_time_;
  delta -= fmod(delta, 0.001);
  
  projector_->command_.current_ = current_setting_;

  if (falling != old_falling_)
  {
    old_falling_ = falling;
    if (falling_edge_pub_ && falling_edge_pub_->trylock())
    {
      //falling_edge_pub_->msg_.stamp = ros::Time(curtime - (stamp - falling) * 1e-6);
      //falling_edge_pub_->msg_.stamp = ros::Time((stamp - falling) * 1e-6);
      falling_edge_pub_->msg_.stamp = ros::Time(delta);
      falling_edge_pub_->unlockAndPublish();
    }
  }
  if (rising != old_rising_)
  {
    old_rising_ = rising;
    if (rising_edge_pub_ && rising_edge_pub_->trylock())
    {
      //rising_edge_pub_->msg_.stamp = ros::Time(curtime - (stamp - rising) * 1e-6);
      //rising_edge_pub_->msg_.stamp = ros::Time((stamp - rising) * 1e-6);
      rising_edge_pub_->msg_.stamp = ros::Time(delta);
      rising_edge_pub_->unlockAndPublish();
    }
  }
}

void ProjectorController::starting()
{
  projector_->command_.enable_ = true;
  projector_->command_.pulse_replicator_ = false;
  //projector_->command_.M_ = 0xf;
  old_rising_ = projector_->state_.rising_timestamp_us_;
  old_falling_ = projector_->state_.falling_timestamp_us_;
  start_time_ = 0;//robot_->getTime().toSec();
}

void ProjectorController::stopping()
{
  projector_->command_.enable_ = false;
  projector_->command_.pulse_replicator_ = true;
  //projector_->command_.M_ = 0x0;
  projector_->command_.current_ = 0;
}

bool ProjectorController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n)
{
  node_handle_ = n;

  assert(robot);
  robot_=robot;

  ROS_DEBUG("ProjectorController::init starting");

  // Get the actuator name.

  if (!n.getParam("actuator", actuator_name_)){
    ROS_ERROR("ProjectorController was not given an actuator.");
    return false;
  }

  rising_edge_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Header>(n, "rising_edge_timestamps", 10));
  falling_edge_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Header>(n, "falling_edge_timestamps", 10));

  projector_ = robot_->model_->hw_->getProjector(actuator_name_);
  ROS_DEBUG("Got projector: %p\n", projector_);
  if (!projector_)
  {
    ROS_ERROR("ProjectorController could not find digital out named \"%s\".",
        actuator_name_.c_str());
    return false;
  } 

  n.param("current", current_setting_, 27.0);
  ROS_DEBUG("Projector current = %f", current_setting_);
    
  return true;
}
