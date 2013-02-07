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

#include "pr2_calibration_controllers/gripper_calibration_controller.h"
#include "ros/time.h"
#include "pluginlib/class_list_macros.h"

using namespace std;
using namespace controller;

PLUGINLIB_EXPORT_CLASS(controller::GripperCalibrationController, pr2_controller_interface::Controller)

namespace controller
{

GripperCalibrationController::GripperCalibrationController()
  : last_publish_time_(0), joint_(NULL)
{
}

GripperCalibrationController::~GripperCalibrationController()
{
}

bool GripperCalibrationController::init(pr2_mechanism_model::RobotState *robot,
                                        ros::NodeHandle &n)
{
  assert(robot);
  robot_ = robot;
  node_ = n;

  node_.param("stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.0001);

  XmlRpc::XmlRpcValue other_joint_names;
  if (node_.getParam("other_joints", other_joint_names))
  {
    if (other_joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("\"other_joints\" was not an array (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    else
    {
      for (int i = 0; i < other_joint_names.size(); ++i)
      {
        pr2_mechanism_model::JointState *j;
        std::string name = (std::string)other_joint_names[i];
        if ((j = robot->getJointState(name))){
          other_joints_.push_back(j);
        }
        else {
          ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",
                    name.c_str(), node_.getNamespace().c_str());
          return false;
        }
      }
    }
  }

  if (!node_.getParam("velocity", search_velocity_))
  {
    ROS_ERROR("No velocity given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  std::string joint_name;
  if (!node_.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(joint_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  std::string actuator_name;
  if (!node_.getParam("actuator", actuator_name))
  {
    ROS_ERROR("No actuator given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(actuator_ = robot->model_->getActuator(actuator_name)))
  {
    ROS_ERROR("Could not find actuator \"%s\" (namespace: %s)",
              actuator_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  bool force_calibration = false;
  node_.getParam("force_calibration", force_calibration);

  state_ = INITIALIZED;
  joint_->calibrated_ = false;
  if (actuator_->state_.zero_offset_ != 0){
    if (force_calibration)
    {
      ROS_INFO("Joint %s will be recalibrated, but was already calibrated at offset %f", 
               joint_name.c_str(), actuator_->state_.zero_offset_);
    }
    else 
    {
      ROS_INFO("Joint %s is already calibrated at offset %f", joint_name.c_str(), actuator_->state_.zero_offset_);
      joint_->calibrated_ = true;
      for (size_t i = 0; i < other_joints_.size(); ++i)
        other_joints_[i]->calibrated_ = true;
      state_ = CALIBRATED;
    }
  }
  else{
    ROS_INFO("Joint %s is not yet calibrated", joint_name.c_str());
  }

  if (!vc_.init(robot, node_))
    return false;

  // advertise service to check calibration
  is_calibrated_srv_ = node_.advertiseService("is_calibrated", &GripperCalibrationController::isCalibrated, this);

  // "Calibrated" topic
  pub_calibrated_.reset(new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

  return true;
}


void GripperCalibrationController::starting()
{
  state_ = INITIALIZED;
  actuator_->state_.zero_offset_ = 0.0;
  joint_->calibrated_ = false;
}


bool GripperCalibrationController::isCalibrated(pr2_controllers_msgs::QueryCalibrationState::Request& req,
						pr2_controllers_msgs::QueryCalibrationState::Response& resp)
{
  resp.is_calibrated = (state_ == CALIBRATED);
  return true;
}


void GripperCalibrationController::update()
{
  assert(joint_);
  assert(actuator_);

  switch (state_)
  {
  case INITIALIZED:
    state_ = BEGINNING;
    return;
  case BEGINNING:
    count_ = 0;
    stop_count_ = 0;
    joint_->calibrated_ = false;
    actuator_->state_.zero_offset_ = 0.0;

    vc_.setCommand(search_velocity_);

    state_ = STARTING;
    break;
  case STARTING:
    // Makes sure we start moving for a bit before checking if we've stopped.
    if (++count_ > 100)
    {
      count_ = 0;
      stop_count_ = 0;
      state_ = CLOSING;
    }
    break;
  case CLOSING:
    // Makes sure the gripper is stopped for a while before cal
    if (fabs(joint_->velocity_) < this->stopped_velocity_tolerance_)
      stop_count_++;
    else
      stop_count_ = 0;

    if (stop_count_ > 100)
    {
      state_ = BACK_OFF;
      stop_count_ = 0;
      vc_.setCommand(-1 * search_velocity_);
    }
    break;
  case BACK_OFF: // Back off so we can reset from a known good position
    if (++stop_count_ > 1000)
    {
      state_ = CLOSING_SLOWLY;
      count_ = 0;
      stop_count_ = 0;
      vc_.setCommand(1.0 * search_velocity_);
    }

    break;
  case CLOSING_SLOWLY: // Close slowly to avoid windup
    // Makes sure the gripper is stopped for a while before cal
    if (fabs(joint_->velocity_) < this->stopped_velocity_tolerance_)
      stop_count_++;
    else
      stop_count_ = 0;

    if (stop_count_ > 500)
    {
      state_ = CALIBRATED;
      actuator_->state_.zero_offset_ = actuator_->state_.position_;
      joint_->calibrated_ = true;
      for (size_t i = 0; i < other_joints_.size(); ++i)
        other_joints_[i]->calibrated_ = true;
      vc_.setCommand(0);
    }

    break;
  case CALIBRATED:
    if (pub_calibrated_) {
      if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime()) {
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
}

