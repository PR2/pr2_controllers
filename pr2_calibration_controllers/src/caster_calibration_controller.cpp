/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#include "pr2_calibration_controllers/caster_calibration_controller.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(controller::CasterCalibrationController, pr2_controller_interface::Controller)

namespace controller {

CasterCalibrationController::CasterCalibrationController()
: robot_(NULL),
  joint_(NULL), wheel_l_joint_(NULL), wheel_r_joint_(NULL), last_publish_time_(0)
{
}

CasterCalibrationController::~CasterCalibrationController()
{
  for (size_t i = 0; i < fake_as.size(); ++i)
    delete fake_as[i];
  for (size_t i = 0; i < fake_js.size(); ++i)
    delete fake_js[i];
}

bool CasterCalibrationController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  node_ = n;
  robot_ = robot;

  // Joints

  std::string joint_name;
  if (!node_.getParam("joints/caster", joint_name))
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
    ROS_ERROR("No calibration reference position specified for joint %s (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("velocity", search_velocity_))
  {
    ROS_ERROR("No velocity given (namespace: %s)", node_.getNamespace().c_str());
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
    ROS_WARN("Negative search velocity is not supported for joint %s. Making the search velocity positve.", joint_name.c_str());
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

  if (!node_.getParam("joints/wheel_l", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(wheel_l_joint_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint %s (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!node_.getParam("joints/wheel_r", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(wheel_r_joint_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint %s (namespace: %s)",
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
  wheel_l_joint_->calibrated_ = false;
  wheel_r_joint_->calibrated_ = false;
  if (actuator_->state_.zero_offset_ != 0){
    if (force_calibration)
    {
      ROS_INFO("Joint %s will be recalibrated, but was already calibrated at offset %f", 
               joint_name.c_str(), actuator_->state_.zero_offset_);
    }
    else 
    {
      ROS_INFO("Joint %s is already calibrated at offset %f", joint_name.c_str(), actuator_->state_.zero_offset_);
      state_ = CALIBRATED;
      joint_->calibrated_ = true;
      wheel_l_joint_->calibrated_ = true;
      wheel_r_joint_->calibrated_ = true;
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

  fake_as.push_back(new pr2_hardware_interface::Actuator);
  fake_js.push_back(new pr2_mechanism_model::JointState);

  if (!cc_.init(robot_, node_))
    return false;

  // advertise service to check calibration
  is_calibrated_srv_ = node_.advertiseService("is_calibrated", &CasterCalibrationController::isCalibrated, this);

  // "Calibrated" topic
  pub_calibrated_.reset(new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

  return true;
}

void CasterCalibrationController::starting()
{
  state_ = INITIALIZED;
  actuator_->state_.zero_offset_ = 0.0;
  joint_->calibrated_ = false;
  wheel_l_joint_->calibrated_ = false;
  wheel_r_joint_->calibrated_ = false;
}


bool CasterCalibrationController::isCalibrated(pr2_controllers_msgs::QueryCalibrationState::Request& req,
					       pr2_controllers_msgs::QueryCalibrationState::Response& resp)
{
  ROS_DEBUG("Is calibrated service %d", state_ == CALIBRATED);
  resp.is_calibrated = (state_ == CALIBRATED);
  return true;
}



void CasterCalibrationController::update()
{
  assert(joint_);
  assert(actuator_);
  ros::Time time = robot_->getTime();

  switch(state_)
  {
  case INITIALIZED:
    cc_.steer_velocity_ = 0.0;
    cc_.drive_velocity_ = 0.0;
    state_ = BEGINNING;
    break;
  case BEGINNING:
    beginning_ = time;
    original_switch_state_ = actuator_->state_.calibration_reading_ & 1;
    original_position_ = joint_->position_;
    cc_.steer_velocity_ = (original_switch_state_ ? -search_velocity_ : search_velocity_);
    state_ = MOVING;
    break;
  case MOVING: {
    bool switch_state_ = actuator_->state_.calibration_reading_ & 1;
    if (switch_state_ != original_switch_state_)
    {
      // detect when we hit the wrong transition because someone pushed the caster during calibration
      if ((cc_.steer_velocity_ > 0.0 && (joint_->position_ - original_position_) < 0) ||
          (cc_.steer_velocity_ < 0.0 && (joint_->position_ - original_position_) > 0))
      {
        state_ = BEGINNING;
        ROS_ERROR("Caster hit the falling edge instead of the rising edge. Calibrating again...");
        ros::Duration(1.0).sleep();  // give caster some time to move away from transition
        break;
      }

      // Where was the joint when the optical switch triggered?
      if (switch_state_ == true)
        actuator_->state_.zero_offset_ = actuator_->state_.last_calibration_rising_edge_;
      else
        actuator_->state_.zero_offset_ = actuator_->state_.last_calibration_falling_edge_;
      
      joint_->calibrated_ = true;
      wheel_l_joint_->calibrated_ = true;
      wheel_r_joint_->calibrated_ = true;
      
      state_ = CALIBRATED;
      cc_.steer_velocity_ = 0.0;
    }
    else
      {
      // The caster is not strong enough to consistently move.  The
      // rest of this block contains the hacks to ensure that
      // calibration always completes.
      if (time > beginning_ + ros::Duration(6.0))
      {
        if ((unstick_iter_ / 1000) % 2 == 0)
          cc_.steer_velocity_ = 4.0 * (original_switch_state_ ? -search_velocity_ : search_velocity_);
        else
          cc_.steer_velocity_ = 0.0;
        ++unstick_iter_;
      }
      else
        unstick_iter_ = 0;
    }
    break;
  }
  case CALIBRATED:
    cc_.steer_velocity_ = 0.0;
    if (pub_calibrated_) {
      if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime())  {
	if (pub_calibrated_->trylock())  {
          last_publish_time_ = robot_->getTime();
	  pub_calibrated_->unlockAndPublish();
	}
      }
    }
    break;
  }

  if (state_ != CALIBRATED)
    cc_.update();
}

} // namespace
