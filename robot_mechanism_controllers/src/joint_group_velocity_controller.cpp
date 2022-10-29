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

#include "robot_mechanism_controllers/joint_group_velocity_controller.h"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS( controller::JointGroupVelocityController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

JointGroupVelocityController::JointGroupVelocityController()
: robot_(NULL), loop_count_(0)
{
}

JointGroupVelocityController::~JointGroupVelocityController()
{
  sub_command_.shutdown();
}

bool JointGroupVelocityController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  using namespace XmlRpc;
  assert(robot);
  node_ = n;
  robot_ = robot;

  // Gets all of the joints
  XmlRpc::XmlRpcValue joint_names;
  if (!node_.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                node_.getNamespace().c_str());
      return false;
    }

    pr2_mechanism_model::JointState *j = robot->getJointState((std::string)name_value);
    if (!j) {
      ROS_ERROR("Joint not found: %s. (namespace: %s)",
                ((std::string)name_value).c_str(), node_.getNamespace().c_str());
      return false;
    }
    joints_.push_back(j);
  }

  // Sets up pid controllers for all of the joints
  std::string gains_ns;
  if (!node_.getParam("gains", gains_ns))
    gains_ns = node_.getNamespace() + "/gains";
  pid_controllers_.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    ros::NodeHandle joint_nh(gains_ns + "/" + joints_[i]->joint_->name);
    if (!pid_controllers_[i].init(joint_nh))
      return false;
  }

  n_joints_ = joint_names.size();

  commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerStateArray>
    (node_, "statearray", 1));
  controller_state_publisher_->lock();
  controller_state_publisher_->msg_.controllestate.resize(joints_.size());
  controller_state_publisher_->unlock();

  sub_command_ = node_.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointGroupVelocityController::setCommandCB, this);

  return true;
}

void JointGroupVelocityController::getGains(control_toolbox::Pid &pid, double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid.getGains(p,i,d,i_max,i_min);
}

std::vector< std::string > JointGroupVelocityController::getJointName()
{
  std::vector< std::string > joint_names;
  for(unsigned int i=0; i<n_joints_; i++)
  {
    joint_names.push_back(joints_[i]->joint_->name);
  }
  return joint_names;
}

// Set the joint velocity commands
void JointGroupVelocityController::setCommand(std::vector<double>  cmd)
{
  commands_buffer_.writeFromNonRT(cmd);
}

// Return the current velocity commands
void JointGroupVelocityController::getCommand(std::vector<double>   & cmd)
{
  cmd = *commands_buffer_.readFromRT();
}

void JointGroupVelocityController::starting()
{
  for (size_t i = 0; i < pid_controllers_.size(); ++i)
    pid_controllers_[i].reset();

}
void JointGroupVelocityController::update()
{
  assert(robot_ != NULL);
  ros::Time time = robot_->getTime();
  ros::Duration dt_ = time - last_time_;
  std::vector<double> & commands = *commands_buffer_.readFromRT();
  std::vector<double> compute_command(n_joints_);
  std::vector<double> compute_error(n_joints_);

  for(unsigned int i=0; i<n_joints_; i++)
  {
    compute_error[i] = commands[i] - joints_[i]->velocity_;
    double command = pid_controllers_[i].computeCommand(compute_error[i], dt_);
    joints_[i]->commanded_effort_ += command;
    compute_command[i] = command;
  }

  if(loop_count_ % 10 == 0){
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      for (unsigned int i = 0; i < n_joints_; ++i)
      {
        pr2_controllers_msgs::JointControllerState singlejointcontrollerstate;
        singlejointcontrollerstate.header.stamp = time;
        singlejointcontrollerstate.set_point = commands[i];
        singlejointcontrollerstate.process_value = joints_[i]->velocity_;
        singlejointcontrollerstate.error = compute_error[i];
        singlejointcontrollerstate.time_step = dt_.toSec();
        singlejointcontrollerstate.command = compute_command[i];

        double dummy;
        getGains(pid_controllers_[i],
                 singlejointcontrollerstate.p,
                 singlejointcontrollerstate.i,
                 singlejointcontrollerstate.d,
                 singlejointcontrollerstate.i_clamp,
                 dummy);
        controller_state_publisher_->msg_.controllestate.push_back(singlejointcontrollerstate);
    }
    controller_state_publisher_->unlockAndPublish();
  }
}

  loop_count_++;

  last_time_ = time;
}

void JointGroupVelocityController::setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  // command_ = msg->data;
  if(msg->data.size()!=n_joints_)
  {
    ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
    return;
  }
  // command_ = msg->data;
  commands_buffer_.writeFromNonRT(msg->data);
}

} // namespace
