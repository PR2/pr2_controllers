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

#include "pr2_calibration_controllers/wrist_calibration_controller.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(controller::WristCalibrationController, pr2_controller_interface::Controller)

namespace controller {


WristCalibrationController::WristCalibrationController()
: robot_(NULL), last_publish_time_(0)
{
}

WristCalibrationController::~WristCalibrationController()
{
  for (size_t i = 0; i < fake_as.size(); ++i)
    delete fake_as[i];
  for (size_t i = 0; i < fake_js.size(); ++i)
    delete fake_js[i];
}

bool WristCalibrationController::init(pr2_mechanism_model::RobotState *robot,
                                      ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;
  robot_ = robot;

  // Joints

  std::string roll_joint_name;
  if (!node_.getParam("roll_joint", roll_joint_name))
  {
    ROS_ERROR("No roll joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(roll_joint_ = robot->getJointState(roll_joint_name)))
  {
    ROS_ERROR("Could not find roll joint \"%s\" (namespace: %s)",
              roll_joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  if (!roll_joint_->joint_->calibration)
  {
    ROS_ERROR("Joint \"%s\" has no calibration reference position specified (namespace: %s)",
              roll_joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }


  std::string flex_joint_name;
  if (!node_.getParam("flex_joint", flex_joint_name))
  {
    ROS_ERROR("No flex joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(flex_joint_ = robot->getJointState(flex_joint_name)))
  {
    ROS_ERROR("Could not find flex joint \"%s\" (namespace: %s)",
              flex_joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  if (!flex_joint_->joint_->calibration)
  {
    ROS_ERROR("Joint \"%s\" has no calibration reference position specified (namespace: %s)",
              flex_joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }




  if (!node_.getParam("roll_velocity", roll_search_velocity_))
  {
    ROS_ERROR("No roll_velocity given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  // check if calibration fields are supported by this controller
  if (!roll_joint_->joint_->calibration->falling && !roll_joint_->joint_->calibration->rising){
    ROS_ERROR("No rising or falling edge is specified for calibration of joint %s. Note that the reference_position is not used any more", roll_joint_name.c_str());
    return false;
  }
  if (roll_joint_->joint_->calibration->falling && roll_joint_->joint_->calibration->rising && roll_joint_->joint_->type != urdf::Joint::CONTINUOUS){
    ROS_ERROR("Both rising and falling edge are specified for non-continuous joint %s. This is not supported.", roll_joint_name.c_str());
    return false;
  }
  if (roll_search_velocity_ < 0){
    roll_search_velocity_ *= -1;
    ROS_ERROR("Negative search velocity is not supported for joint %s. Making the search velocity positve.", roll_joint_name.c_str());
  }

  // sets reference position
  if (roll_joint_->joint_->calibration->falling && roll_joint_->joint_->calibration->rising){
    roll_joint_->reference_position_ = *(roll_joint_->joint_->calibration->rising);
  }
  else if (roll_joint_->joint_->calibration->falling){
    roll_joint_->reference_position_ = *(roll_joint_->joint_->calibration->falling);
  }
  else if (roll_joint_->joint_->calibration->rising){
    roll_joint_->reference_position_ = *(roll_joint_->joint_->calibration->rising);
  }


  if (!node_.getParam("flex_velocity", flex_search_velocity_))
  {
    ROS_ERROR("No flex_velocity given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  // check if calibration fields are supported by this controller
  if (!flex_joint_->joint_->calibration->falling && !flex_joint_->joint_->calibration->rising){
    ROS_ERROR("No rising or falling edge is specified for calibration of joint %s. Note that the reference_position is not used any more", flex_joint_name.c_str());
    return false;
  }
  if (flex_joint_->joint_->calibration->falling && flex_joint_->joint_->calibration->rising && flex_joint_->joint_->type != urdf::Joint::CONTINUOUS){
    ROS_ERROR("Both rising and falling edge are specified for non-continuous joint %s. This is not supported.", flex_joint_name.c_str());
    return false;
  }
  if (flex_search_velocity_ < 0){
    flex_search_velocity_ *= -1;
    ROS_ERROR("Negative search velocity is not supported for joint %s. Making the search velocity positve.", flex_joint_name.c_str());
  }

  // sets reference position
  if (flex_joint_->joint_->calibration->falling && flex_joint_->joint_->calibration->rising){
    flex_joint_->reference_position_ = *(flex_joint_->joint_->calibration->rising);
  }
  else if (flex_joint_->joint_->calibration->falling){
    flex_joint_->reference_position_ = *(flex_joint_->joint_->calibration->falling);
  }
  else if (flex_joint_->joint_->calibration->rising){
    flex_joint_->reference_position_ = *(flex_joint_->joint_->calibration->rising);
  }

  // Actuators
  std::string actuator_l_name;
  if (!node_.getParam("actuator_l", actuator_l_name))
  {
    ROS_ERROR("No actuator_l given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(actuator_l_ = robot->model_->getActuator(actuator_l_name)))
  {
    ROS_ERROR("Could not find actuator \"%s\" (namespace: %s)",
              actuator_l_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  std::string actuator_r_name;
  if (!node_.getParam("actuator_r", actuator_r_name))
  {
    ROS_ERROR("No actuator_r given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(actuator_r_ = robot->model_->getActuator(actuator_r_name)))
  {
    ROS_ERROR("Could not find actuator \"%s\" (namespace: %s)",
              actuator_r_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  bool force_calibration = false;
  node_.getParam("force_calibration", force_calibration);

  roll_joint_->calibrated_ = false;
  flex_joint_->calibrated_ = false;
  state_ = INITIALIZED;
  if (actuator_l_->state_.zero_offset_ != 0 && actuator_l_->state_.zero_offset_ != 0){
    if (force_calibration)
    {
      ROS_INFO("Joints %s and %s are already calibrated but will be recalibrated. "
               "Actuator %s was zeroed at %f and %s was zeroed at %f.", 
               flex_joint_name.c_str(), roll_joint_name.c_str(),
               actuator_r_->name_.c_str(), actuator_r_->state_.zero_offset_,
               actuator_l_->name_.c_str(), actuator_l_->state_.zero_offset_
               );
    }
    else 
    {
      ROS_INFO("Wrist joints %s and %s are already calibrated", flex_joint_name.c_str(), roll_joint_name.c_str());
      flex_joint_->calibrated_ = true;
      roll_joint_->calibrated_ = true;
      state_ = CALIBRATED;
    }
  }
  else{
    ROS_INFO("Not both wrist joints %s and %s are are calibrated. Will re-calibrate both of them", flex_joint_name.c_str(), roll_joint_name.c_str());
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
    ROS_ERROR("Could not find transmission \"%s\" (namespace: %s)",
              transmission_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  // Prepares the namespaces for the velocity controllers

  XmlRpc::XmlRpcValue pid;
  node_.getParam("pid", pid);

  ros::NodeHandle roll_node(node_, "roll_velocity");
  roll_node.setParam("type", std::string("JointVelocityController"));
  roll_node.setParam("joint", roll_joint_name);
  roll_node.setParam("pid", pid);

  ros::NodeHandle flex_node(node_, "flex_velocity");
  flex_node.setParam("type", std::string("JointVelocityController"));
  flex_node.setParam("joint", flex_joint_name);
  flex_node.setParam("pid", pid);

  fake_as.push_back(new pr2_hardware_interface::Actuator);
  fake_as.push_back(new pr2_hardware_interface::Actuator);
  fake_js.push_back(new pr2_mechanism_model::JointState);
  fake_js.push_back(new pr2_mechanism_model::JointState);

  const int LEFT_MOTOR = pr2_mechanism_model::WristTransmission::LEFT_MOTOR;
  const int RIGHT_MOTOR = pr2_mechanism_model::WristTransmission::RIGHT_MOTOR;
  const int FLEX_JOINT = pr2_mechanism_model::WristTransmission::FLEX_JOINT;
  const int ROLL_JOINT = pr2_mechanism_model::WristTransmission::ROLL_JOINT;
  fake_js[FLEX_JOINT]->joint_ = flex_joint_->joint_;
  fake_js[ROLL_JOINT]->joint_ = roll_joint_->joint_;

  if (!vc_roll_.init(robot_, roll_node)) return false;
  if (!vc_flex_.init(robot_, flex_node)) return false;

  // advertise service to check calibration
  is_calibrated_srv_ = node_.advertiseService("is_calibrated", &WristCalibrationController::isCalibrated, this);

  // "Calibrated" topic
  pub_calibrated_.reset(new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

  return true;
}


void WristCalibrationController::starting()
{
  state_ = INITIALIZED;
  actuator_r_->state_.zero_offset_ = 0.0;
  actuator_l_->state_.zero_offset_ = 0.0;
  flex_joint_->calibrated_ = false;
  roll_joint_->calibrated_ = false;
}


bool WristCalibrationController::isCalibrated(pr2_controllers_msgs::QueryCalibrationState::Request& req,
					      pr2_controllers_msgs::QueryCalibrationState::Response& resp)
{
  ROS_DEBUG("Is calibrated service %d", state_ == CALIBRATED);
  resp.is_calibrated = (state_ == CALIBRATED);
  return true;
}


void WristCalibrationController::update()
{
  // Flex optical switch is connected to actuator_l
  // Roll optical switch is connected to actuator_r

  switch(state_)
  {
  case INITIALIZED:
    actuator_l_->state_.zero_offset_ = 0;
    actuator_r_->state_.zero_offset_ = 0;
    flex_joint_->calibrated_ = false;
    roll_joint_->calibrated_ = false;
    vc_flex_.setCommand(0);
    vc_roll_.setCommand(0);
    state_ = BEGINNING;
    break;
  case BEGINNING:
    vc_roll_.setCommand(0);
    // Because of the huge hysteresis on the wrist flex calibration sensor, we want to move towards the
    // high side for a long period of time, regardless of which side we start on
    countdown_ = 1000;
    if (actuator_l_->state_.calibration_reading_)
      state_ = MOVING_FLEX_TO_HIGH;
    else
      state_ = MOVING_FLEX_TO_HIGH;
    break;
  case MOVING_FLEX_TO_HIGH:
    vc_flex_.setCommand(-flex_search_velocity_);

    if (actuator_l_->state_.calibration_reading_)
    {
      if (--countdown_ <= 0)
        state_ = MOVING_FLEX;
    }
    else
      countdown_ = 1000;
    break;
  case MOVING_FLEX: {
    // Calibrates across the falling edge in the positive joint direction.
    vc_flex_.setCommand(flex_search_velocity_);
    if (actuator_l_->state_.calibration_reading_ == false)
    {
      flex_switch_l_ = actuator_l_->state_.last_calibration_falling_edge_;

      // But where was actuator_r at the transition?  Unfortunately,
      // actuator_r is not connected to the flex joint's optical
      // switch, so we don't know directly.  Instead, we estimate
      // actuator_r's position based on the switch position of
      // actuator_l.
      double dl = actuator_l_->state_.position_ - prev_actuator_l_position_;
      double dr = actuator_r_->state_.position_ - prev_actuator_r_position_;
      double k = (flex_switch_l_ - prev_actuator_l_position_) / dl;
      if (dl == 0)
      {
        // This might be a serious hardware failure, so we're going to break realtime.
        ROS_WARN("Left actuator (flex joint) didn't move even though the calibration flag tripped.  "
                 "This may indicate an encoder problem. (namespace: %s",
                 node_.getNamespace().c_str());
        k = 0.5;
      }
      else if ( !(0 <= k && k <= 1) )
      {
        // This is really serious, so we're going to break realtime to report it.
        ROS_ERROR("k = %.4lf is outside of [0,1].  This probably indicates a hardware failure "
                  "on the left actuator or the flex joint.  dl = %.4lf, dr = %.4lf, prev = %.4lf.  "
                  "Broke realtime to report (namespace: %s)",
                  k, dl, dr, prev_actuator_l_position_, node_.getNamespace().c_str());
        state_ = INITIALIZED;
        break;
      }

      flex_switch_r_ = k * dr + prev_actuator_r_position_;

      //original_switch_state_ = actuator_r_->state_.calibration_reading_;

      // Now we calibrate the roll joint
      vc_flex_.setCommand(0);
      if (actuator_r_->state_.calibration_reading_)
        state_ = MOVING_ROLL_TO_LOW;
      else
        state_ = MOVING_ROLL;
    }
    break;
  }
  case MOVING_ROLL_TO_LOW:
    vc_roll_.setCommand(roll_search_velocity_);
    if (actuator_r_->state_.calibration_reading_ == false)
      state_ = MOVING_ROLL;
    break;
  case MOVING_ROLL: {
    // Calibrates across the rising edge in the positive joint direction.
    vc_roll_.setCommand(roll_search_velocity_);
    if (actuator_r_->state_.calibration_reading_)
    {
      roll_switch_r_ = actuator_r_->state_.last_calibration_rising_edge_;

      // See corresponding comment above.
      double dl = actuator_l_->state_.position_ - prev_actuator_l_position_;
      double dr = actuator_r_->state_.position_ - prev_actuator_r_position_;
      double k = (roll_switch_r_ - prev_actuator_r_position_) / dr;
      if (dr == 0)
      {
        // This might be a serious hardware failure, so we're going to break realtime.
        ROS_WARN("Right actuator (roll joint) didn't move even though the calibration flag tripped.  "
                 "This may indicate an encoder problem. (namespace: %s",
                 node_.getNamespace().c_str());
        k = 0.5;
      }
      else if ( !(0 <= k && k <= 1) )
      {
        // This is really serious, so we're going to break realtime to report it.
        ROS_ERROR("k = %.4lf is outside of [0,1].  This probably indicates a hardware failure "
                  "on the right actuator or the roll joint.  dl = %.4lf, dr = %.4lf, prev = %.4lf.  "
                  "Broke realtime to report (namespace: %s)",
                  k, dl, dr, prev_actuator_r_position_, node_.getNamespace().c_str());
        state_ = INITIALIZED;
        break;
      }
      roll_switch_l_ =  k * dl + prev_actuator_l_position_;


      //----------------------------------------------------------------------
      //       Calibration computation
      //----------------------------------------------------------------------

      // At this point, we know the actuator positions when the
      // optical switches were hit.  Now we compute the actuator
      // positions when the joints should be at 0.

      const int LEFT_MOTOR = pr2_mechanism_model::WristTransmission::LEFT_MOTOR;
      const int RIGHT_MOTOR = pr2_mechanism_model::WristTransmission::RIGHT_MOTOR;
      const int FLEX_JOINT = pr2_mechanism_model::WristTransmission::FLEX_JOINT;
      const int ROLL_JOINT = pr2_mechanism_model::WristTransmission::ROLL_JOINT;

      // Finds the (uncalibrated) joint position where the flex optical switch triggers
      fake_as[LEFT_MOTOR]->state_.position_ = flex_switch_l_;
      fake_as[RIGHT_MOTOR]->state_.position_ = flex_switch_r_;
      transmission_->propagatePosition(fake_as, fake_js);
      double flex_joint_switch = fake_js[FLEX_JOINT]->position_;

      // Finds the (uncalibrated) joint position where the roll optical switch triggers
      fake_as[LEFT_MOTOR]->state_.position_ = roll_switch_l_;
      fake_as[RIGHT_MOTOR]->state_.position_ = roll_switch_r_;
      transmission_->propagatePosition(fake_as, fake_js);
      double roll_joint_switch = fake_js[ROLL_JOINT]->position_;

      // Finds the (uncalibrated) joint position at the desired zero
      fake_js[FLEX_JOINT]->position_ = flex_joint_switch;
      fake_js[ROLL_JOINT]->position_ = roll_joint_switch;

      // Determines the actuator zero position from the desired joint zero positions
      transmission_->propagatePositionBackwards(fake_js, fake_as);
      if (std::isnan(fake_as[LEFT_MOTOR]->state_.position_) ||
          std::isnan(fake_as[RIGHT_MOTOR]->state_.position_))
      {
        ROS_ERROR("Restarting calibration because a computed offset was NaN. If this happens "
                  "repeatedly it may indicate a hardware failure.  (namespace: %s)",
                  node_.getNamespace().c_str());
        state_ = INITIALIZED;
        break;
      }
      actuator_l_->state_.zero_offset_ = fake_as[LEFT_MOTOR]->state_.position_;
      actuator_r_->state_.zero_offset_ = fake_as[RIGHT_MOTOR]->state_.position_;

      flex_joint_->calibrated_ = true;
      roll_joint_->calibrated_ = true;
      state_ = CALIBRATED;

      vc_flex_.setCommand(0);
      vc_roll_.setCommand(0);
    }

    break;
  }
  case CALIBRATED:
    if (pub_calibrated_) {
      if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime()) {
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
  {
    vc_flex_.update();
    vc_roll_.update();
  }

  prev_actuator_l_position_ = actuator_l_->state_.position_;
  prev_actuator_r_position_ = actuator_r_->state_.position_;
}
}

