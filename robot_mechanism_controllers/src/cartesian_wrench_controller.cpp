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

#include "robot_mechanism_controllers/cartesian_wrench_controller.h"
#include <algorithm>
#include "pluginlib/class_list_macros.h"


using namespace KDL;

PLUGINLIB_EXPORT_CLASS( controller::CartesianWrenchController, pr2_controller_interface::Controller)

namespace controller {

CartesianWrenchController::CartesianWrenchController()
: robot_state_(NULL),
  jnt_to_jac_solver_(NULL)
{}



CartesianWrenchController::~CartesianWrenchController()
{
  sub_command_.shutdown();
}


bool CartesianWrenchController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  // test if we got robot pointer
  assert(robot);
  robot_state_ = robot;

  node_ = n;

  // get name of root and tip from the parameter server
  std::string root_name, tip_name;
  if (!node_.getParam("root_name", root_name)){
    ROS_ERROR("CartesianWrenchController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tip_name", tip_name)){
    ROS_ERROR("CartesianWrenchController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // create robot chain from root to tip
  if (!chain_.init(robot_state_, root_name, tip_name)){
    ROS_ERROR("Initializing chain from %s to %s failed", root_name.c_str(), tip_name.c_str());
    return false;
  }
  if (!chain_.allCalibrated())
  {
    ROS_ERROR("Not all joints in the chain are calibrated (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  chain_.toKDL(kdl_chain_);

  // create solver
  jnt_to_jac_solver_.reset(new ChainJntToJacSolver(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_.resize(kdl_chain_.getNrOfJoints());
  jacobian_.resize(kdl_chain_.getNrOfJoints());


  // subscribe to wrench commands
  sub_command_ = node_.subscribe<geometry_msgs::Wrench>
    ("command", 1, &CartesianWrenchController::command, this);

  return true;
}

void CartesianWrenchController::starting()
{
  // set desired wrench to 0
  wrench_desi_ = Wrench::Zero();
}



void CartesianWrenchController::update()
{
  // check if joints are calibrated
  if (!chain_.allCalibrated()){
    return;
  }

  // get joint positions
  chain_.getPositions(jnt_pos_);

  // get the chain jacobian
  jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);

  // convert the wrench into joint efforts
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
    jnt_eff_(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_eff_(i) += (jacobian_(j,i) * wrench_desi_(j));
  }

  // set effort to joints
  chain_.setEfforts(jnt_eff_);
}



void CartesianWrenchController::command(const geometry_msgs::WrenchConstPtr& wrench_msg)
{
  // convert to wrench command
  wrench_desi_.force(0) = wrench_msg->force.x;
  wrench_desi_.force(1) = wrench_msg->force.y;
  wrench_desi_.force(2) = wrench_msg->force.z;
  wrench_desi_.torque(0) = wrench_msg->torque.x;
  wrench_desi_.torque(1) = wrench_msg->torque.y;
  wrench_desi_.torque(2) = wrench_msg->torque.z;
}

}; // namespace
