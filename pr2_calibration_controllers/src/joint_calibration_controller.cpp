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

PLUGINLIB_REGISTER_CLASS(JointCalibrationController, controller::JointCalibrationController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

JointCalibrationController::JointCalibrationController()
: robot_(NULL), last_publish_time_(0), state_(INITIALIZED),
  actuator_(NULL), joint_(NULL), transmission_(NULL)
{
}

JointCalibrationController::~JointCalibrationController()
{
}

bool JointCalibrationController::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  assert(config);

  TiXmlElement *cal = config->FirstChildElement("calibrate");
  if (!cal)
  {
    std::cerr<<"JointCalibrationController was not given calibration parameters"<<std::endl;
    return false;
  }

  if(cal->QueryDoubleAttribute("velocity", &search_velocity_) != TIXML_SUCCESS)
  {
    std::cerr<<"Velocity value was not specified\n";
    return false;
  }

  const char *joint_name = cal->Attribute("joint");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "Error: JointCalibrationController could not find joint \"%s\"\n",
            joint_name);
    return false;
  }

  const char *actuator_name = cal->Attribute("actuator");
  actuator_ = actuator_name ? robot->model_->getActuator(actuator_name) : NULL;
  if (!actuator_)
  {
    fprintf(stderr, "Error: JointCalibrationController could not find actuator \"%s\"\n",
            actuator_name);
    return false;
  }

  const char *transmission_name = cal->Attribute("transmission");
  transmission_ = transmission_name ? robot->model_->getTransmission(transmission_name) : NULL;
  if (!transmission_)
  {
    fprintf(stderr, "Error: JointCalibrationController could not find transmission \"%s\"\n",
            transmission_name);
    return false;
  }

  control_toolbox::Pid pid;
  TiXmlElement *pid_el = config->FirstChildElement("pid");
  if (!pid_el)
  {
    fprintf(stderr, "Error: JointCalibrationController was not given a pid element.\n");
    return false;
  }
  if (!pid.initXml(pid_el))
    return false;

  if (!vc_.init(robot, joint_name, pid))
    return false;

  return true;
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

  // Contained velocity controller

  if (!vc_.init(robot, node_))
    return false;

  // "Calibrated" topic
  pub_calibrated_.reset(
    new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

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
    if (actuator_->state_.calibration_reading_)
      state_ = MOVING_TO_LOW;
    else
      state_ = MOVING_TO_HIGH;
    break;
  case MOVING_TO_LOW:
    vc_.setCommand(-search_velocity_);
    if (!actuator_->state_.calibration_reading_)
    {
      if (--countdown_ <= 0)
        state_ = MOVING_TO_HIGH;
    }
    else
      countdown_ = 200;
    break;
  case MOVING_TO_HIGH: {
    vc_.setCommand(search_velocity_);

    if (actuator_->state_.calibration_reading_)
    {
      pr2_hardware_interface::Actuator a;
      pr2_mechanism_model::JointState j;
      std::vector<pr2_hardware_interface::Actuator*> fake_a;
      std::vector<pr2_mechanism_model::JointState*> fake_j;
      fake_a.push_back(&a);
      fake_j.push_back(&j);

      fake_a[0]->state_.position_ = actuator_->state_.last_calibration_rising_edge_;
      transmission_->propagatePosition(fake_a, fake_j);



      // Where was the joint when the optical switch triggered?
      fake_a[0]->state_.position_ = actuator_->state_.last_calibration_rising_edge_;
      transmission_->propagatePosition(fake_a, fake_j);

      // What is the actuator position at the joint's zero?
      assert(joint_->joint_->calibration);
      fake_j[0]->position_ = fake_j[0]->position_ - joint_->joint_->calibration->reference_position;
      transmission_->propagatePositionBackwards(fake_j, fake_a);

      actuator_->state_.zero_offset_ = fake_a[0]->state_.position_;
      joint_->calibrated_ = true;

      state_ = CALIBRATED;
      vc_.setCommand(0.0);
    }
    break;
  }
  case CALIBRATED:
    if (pub_calibrated_)
    {
      if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime())
      {
        assert(pub_calibrated_);
        if (pub_calibrated_->trylock())
        {
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
