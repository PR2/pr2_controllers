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
 * Author: Wim Meeussen
 */

#include "robot_mechanism_controllers/cartesian_twist_controller.h"
#include <algorithm>
#include "kdl/chainfksolvervel_recursive.hpp"
#include "pluginlib/class_list_macros.h"

using namespace KDL;

PLUGINLIB_EXPORT_CLASS( controller::CartesianTwistController, pr2_controller_interface::Controller)

namespace controller {


CartesianTwistController::CartesianTwistController()
  : robot_state_(NULL),
    jnt_to_twist_solver_(NULL)
{}



CartesianTwistController::~CartesianTwistController()
{
  sub_command_.shutdown();
}


bool CartesianTwistController::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &n)
{
  node_ = n;

  // get name of root and tip from the parameter server
  std::string root_name, tip_name;
  if (!node_.getParam("root_name", root_name)){
    ROS_ERROR("CartesianTwistController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tip_name", tip_name)){
    ROS_ERROR("CartesianTwistController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!chain_.init(robot_state, root_name, tip_name))
    return false;
  if (!chain_.allCalibrated())
  {
    ROS_ERROR("Not all joints in the chain are calibrated (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  chain_.toKDL(kdl_chain_);

  // create solver
  jnt_to_twist_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
  jac_solver_.reset(new ChainJntToJacSolver(kdl_chain_));
  jnt_posvel_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_.resize(kdl_chain_.getNrOfJoints());
  jacobian_.resize(kdl_chain_.getNrOfJoints());

  // constructs 3 identical pid controllers: for the x,y and z translations
  control_toolbox::Pid pid_controller;
  if (!pid_controller.init(ros::NodeHandle(node_, "fb_trans"))) return false;
  for (unsigned int i=0; i<3; i++)
    fb_pid_controller_.push_back(pid_controller);

  // constructs 3 identical pid controllers: for the x,y and z rotations
  if (!pid_controller.init(ros::NodeHandle(node_, "fb_rot"))) return false;
  for (unsigned int i=0; i<3; i++)
    fb_pid_controller_.push_back(pid_controller);

  // get parameters
  node_.param("ff_trans", ff_trans_, 0.0) ;
  node_.param("ff_rot", ff_rot_, 0.0) ;

  // subscribe to twist commands
  sub_command_ = node_.subscribe<geometry_msgs::Twist>
    ("command", 1, &CartesianTwistController::command, this);

  return true;
}

void CartesianTwistController::starting()
{
  // reset pid controllers
  for (unsigned int i=0; i<6; i++)
    fb_pid_controller_[i].reset();

  // time
  last_time_ = robot_state_->getTime();

  // set disired twist to 0
  twist_desi_ = Twist::Zero();
}



void CartesianTwistController::update()
{
  // check if joints are calibrated
  if (!chain_.allCalibrated())
    return;

  // get time
  ros::Time time = robot_state_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  // get the joint positions and velocities
  chain_.getVelocities(jnt_posvel_);

  // get cartesian twist error
  FrameVel twist;
  jnt_to_twist_solver_->JntToCart(jnt_posvel_, twist);
  twist_meas_ = twist.deriv();
  Twist error = twist_desi_ - twist_meas_;

  // get the chain jacobian
  jac_solver_->JntToJac(jnt_posvel_.q, jacobian_);

  // pid feedback
  for (unsigned int i=0; i<3; i++)
    wrench_out_.force(i) = (twist_desi_.vel(i) * ff_trans_) + fb_pid_controller_[i].computeCommand(error.vel(i), dt);

  for (unsigned int i=0; i<3; i++)
    wrench_out_.torque(i) = (twist_desi_.rot(i) * ff_rot_) + fb_pid_controller_[i+3].computeCommand(error.rot(i), dt);

  // Converts the wrench into joint efforts with a jacbobian-transpose
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
    jnt_eff_(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_eff_(i) += (jacobian_(j,i) * wrench_out_(j));
  }

  // set effort to joints
  chain_.addEfforts(jnt_eff_);
}


void CartesianTwistController::command(const geometry_msgs::TwistConstPtr& twist_msg)
{
  // convert to twist command
  twist_desi_.vel(0) = twist_msg->linear.x;
  twist_desi_.vel(1) = twist_msg->linear.y;
  twist_desi_.vel(2) = twist_msg->linear.z;
  twist_desi_.rot(0) = twist_msg->angular.x;
  twist_desi_.rot(1) = twist_msg->angular.y;
  twist_desi_.rot(2) = twist_msg->angular.z;
}

} // namespace
