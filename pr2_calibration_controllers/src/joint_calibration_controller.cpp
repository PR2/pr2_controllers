/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include "pr2_calibration_controllers/joint_calibration_controller.h"
#include "ros/time.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(controller::JointCalibrationController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

JointCalibrationController::JointCalibrationController()
: robot_(NULL), last_publish_time_(0),
  actuator_(NULL), joint_(NULL)
{
}

JointCalibrationController::~JointCalibrationController()
{
}

bool JointCalibrationController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  robot_ = robot;
  node_ = n;

  // Joint
  std::string joint_name;
  if (!node_.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(joint_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint %s (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  if (!joint_->joint_->calibration)
  {
    ROS_ERROR("Joint %s has no calibration reference position specified (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  // Actuator
  std::string actuator_name;
  if (!node_.getParam("actuator", actuator_name))
  {
    ROS_ERROR("No actuator given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(actuator_ = robot->model_->getActuator(actuator_name)))
  {
    ROS_ERROR("Could not find actuator %s (namespace: %s)",
              actuator_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  bool force_calibration = false;
  node_.getParam("force_calibration", force_calibration);

  state_ = INITIALIZED;
  joint_->calibrated_ = false;
  if (actuator_->state_.zero_offset_ != 0) 
  {
    if (force_calibration)
    {
      ROS_INFO("Joint %s will be recalibrated, but was already calibrated at offset %f", 
               joint_name.c_str(), actuator_->state_.zero_offset_);
    }
    else
    {
      ROS_INFO("Joint %s is already calibrated at offset %f", 
               joint_name.c_str(), actuator_->state_.zero_offset_);
      state_ = CALIBRATED;
      joint_->calibrated_ = true;
    }
  }
  else{
    ROS_INFO("Joint %s is not yet calibrated", joint_name.c_str());
  }

  // Transmission
  std::string transmission_name;
  if (!node_.getParam("transmission", transmission_name))
  {
    ROS_ERROR("No transmission given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(transmission_ = robot->model_->getTransmission(transmission_name)))
  {
    ROS_ERROR("Could not find transmission %s (namespace: %s)",
              transmission_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!node_.getParam("velocity", search_velocity_))
  {
    ROS_ERROR("Velocity value was not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  // check if calibration fields are supported by this controller
  if (!joint_->joint_->calibration->falling && !joint_->joint_->calibration->rising){
    ROS_ERROR("No rising or falling edge is specified for calibration of joint %s. Note that the reference_position is not used any more", joint_name.c_str());
    return false;
  }
  if (joint_->joint_->calibration->falling && joint_->joint_->calibration->rising && joint_->joint_->type != urdf::Joint::CONTINUOUS){
    ROS_ERROR("Both rising and falling edge are specified for non-continuous joint %s. This is not supported.", joint_name.c_str());
    return false;
  }
  if (search_velocity_ < 0){
    search_velocity_ *= -1;
    ROS_ERROR("Negative search velocity is not supported for joint %s. Making the search velocity positive.", joint_name.c_str());
  }

  // finds search velocity based on rising or falling edge
  if (joint_->joint_->calibration->falling && joint_->joint_->calibration->rising){
    joint_->reference_position_ = *(joint_->joint_->calibration->rising);
    ROS_DEBUG("Using positive search velocity for joint %s", joint_name.c_str());
  }
  else if (joint_->joint_->calibration->falling){
    joint_->reference_position_ = *(joint_->joint_->calibration->falling);
    search_velocity_ *= -1.0;
    ROS_DEBUG("Using negative search velocity for joint %s", joint_name.c_str());
  }
  else if (joint_->joint_->calibration->rising){
    joint_->reference_position_ = *(joint_->joint_->calibration->rising);
    ROS_DEBUG("Using positive search velocity for joint %s", joint_name.c_str());
  }


  // Contained velocity controller
  if (!vc_.init(robot, node_))
    return false;

  // advertise service to check calibration
  is_calibrated_srv_ = node_.advertiseService("is_calibrated", &JointCalibrationController::isCalibrated, this);

  // "Calibrated" topic
  pub_calibrated_.reset(new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));


  return true;
}


void JointCalibrationController::starting()
{
  state_ = INITIALIZED;
  joint_->calibrated_ = false;
  actuator_->state_.zero_offset_ = 0.0;
}


bool JointCalibrationController::isCalibrated(pr2_controllers_msgs::QueryCalibrationState::Request& req,
					      pr2_controllers_msgs::QueryCalibrationState::Response& resp)
{
  ROS_DEBUG("Is calibrated service %d", state_ == CALIBRATED);
  resp.is_calibrated = (state_ == CALIBRATED);
  return true;
}


void JointCalibrationController::update()
{
  assert(joint_);
  assert(actuator_);

  switch(state_)
  {
  case INITIALIZED:
    vc_.setCommand(0.0);
    state_ = BEGINNING;
    break;
  case BEGINNING:
    if (actuator_->state_.calibration_reading_ & 1)
      state_ = MOVING_TO_LOW;
    else{
      state_ = MOVING_TO_HIGH;
      original_position_ = joint_->position_;
    }
    break;
  case MOVING_TO_LOW:
    vc_.setCommand(-search_velocity_);
    if (!(actuator_->state_.calibration_reading_ & 1))
    {
      if (--countdown_ <= 0){
        state_ = MOVING_TO_HIGH;
	original_position_ = joint_->position_;
      }
    }
    else
      countdown_ = 200;
    break;
  case MOVING_TO_HIGH: {
    vc_.setCommand(search_velocity_);

    if (actuator_->state_.calibration_reading_ & 1)
    {
      // detect when we hit the wrong transition because someone pushed the joint during calibration
      if ((search_velocity_ > 0.0 && (joint_->position_ - original_position_) < 0) ||
	  (search_velocity_ < 0.0 && (joint_->position_ - original_position_) > 0))
	{
	  state_ = BEGINNING;
	  ROS_ERROR("Joint hit the falling edge instead of the rising edge. Calibrating again...");
	  ros::Duration(1.0).sleep();  // give joint some time to move away from transition
	  break;
	}

      pr2_hardware_interface::Actuator a;
      pr2_mechanism_model::JointState j;

      // store position of flag in actuator 
      actuator_->state_.zero_offset_ = actuator_->state_.last_calibration_rising_edge_;

      // mark the joint as calibrated
      joint_->calibrated_ = true;

      state_ = CALIBRATED;
      vc_.setCommand(0.0);
    }
    break;
  }
  case CALIBRATED:
    if (pub_calibrated_) {
      if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime()){
	assert(pub_calibrated_);
	if (pub_calibrated_->trylock()) {
	  last_publish_time_ = robot_->getTime();
	  pub_calibrated_->unlockAndPublish();
	}
      }
    }
    break;
  }

  if (state_ != CALIBRATED)
    vc_.update();
}
} // namespace
