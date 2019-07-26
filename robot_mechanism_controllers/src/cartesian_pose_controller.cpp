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


#include <boost/bind.hpp>

#include "robot_mechanism_controllers/cartesian_pose_controller.h"
#include <algorithm>
#include "kdl/chainfksolverpos_recursive.hpp"
#include "pluginlib/class_list_macros.h"
#include "tf_conversions/tf_kdl.h"


using namespace KDL;
using namespace tf;
using namespace std;

PLUGINLIB_EXPORT_CLASS( controller::CartesianPoseController, pr2_controller_interface::Controller)

namespace controller {

CartesianPoseController::CartesianPoseController()
: robot_state_(NULL)
{}

CartesianPoseController::~CartesianPoseController()
{
  command_filter_.reset();
}


bool CartesianPoseController::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &n)
{
  node_ = n;

  // get name of root and tip from the parameter server
  std::string tip_name;
  if (!node_.getParam("root_name", root_name_)){
    ROS_ERROR("CartesianPoseController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tip_name", tip_name)){
    ROS_ERROR("CartesianPoseController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!chain_.init(robot_state_, root_name_, tip_name))
    return false;
  if (!chain_.allCalibrated())
  {
    ROS_ERROR("Not all joints in the chain are calibrated (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  chain_.toKDL(kdl_chain_);

  // create solver
  jnt_to_pose_solver_.reset(new ChainFkSolverPos_recursive(kdl_chain_));
  jac_solver_.reset(new ChainJntToJacSolver(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_.resize(kdl_chain_.getNrOfJoints());
  jacobian_.resize(kdl_chain_.getNrOfJoints());

  // create pid controller for the translation and for the rotation
  control_toolbox::Pid pid_controller;
  if (!pid_controller.init(ros::NodeHandle(node_,"fb_trans"))) return false;
  for (unsigned int i = 0; i < 3; i++)
    pid_controller_.push_back(pid_controller);
  if (!pid_controller.init(ros::NodeHandle(node_,"fb_rot"))) return false;
  for (unsigned int i = 0; i < 3; i++)
    pid_controller_.push_back(pid_controller);

  // subscribe to pose commands
  sub_command_.subscribe(node_, "command", 10);
  command_filter_.reset(new tf::MessageFilter<geometry_msgs::PoseStamped>(
                          sub_command_, tf_, root_name_, 10, node_));
  command_filter_->registerCallback(boost::bind(&CartesianPoseController::command, this, _1));

  // realtime publisher for control state
  state_error_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(node_, "state/error", 1));
  state_pose_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node_, "state/pose", 1));

  return true;
}

void CartesianPoseController::starting()
{
  // reset pid controllers
  for (unsigned int i=0; i<6; i++)
    pid_controller_[i].reset();

  // initialize desired pose/twist
  twist_ff_ = Twist::Zero();
  pose_desi_ = getPose();
  last_time_ = robot_state_->getTime();

  loop_count_ = 0;
}



void CartesianPoseController::update()
{
  // get time
  ros::Time time = robot_state_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  // get current pose
  pose_meas_ = getPose();

  // pose feedback into twist
  twist_error_ = diff(pose_meas_, pose_desi_);
  KDL::Wrench wrench_desi;
  for (unsigned int i=0; i<6; i++)
    wrench_desi(i) = pid_controller_[i].computeCommand(twist_error_(i), dt);

  // get the chain jacobian
  jac_solver_->JntToJac(jnt_pos_, jacobian_);

  // Converts the wrench into joint efforts with a jacbobian-transpose
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
    jnt_eff_(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_eff_(i) += (jacobian_(j,i) * wrench_desi(j));
  }

  // set effort to joints
  chain_.addEfforts(jnt_eff_);


  if (++loop_count_ % 100 == 0){
    if (state_error_publisher_){
      if (state_error_publisher_->trylock()){
        state_error_publisher_->msg_.linear.x = twist_error_.vel(0);
        state_error_publisher_->msg_.linear.y = twist_error_.vel(1);
        state_error_publisher_->msg_.linear.z = twist_error_.vel(2);
        state_error_publisher_->msg_.angular.x = twist_error_.rot(0);
	state_error_publisher_->msg_.angular.y = twist_error_.rot(1);
        state_error_publisher_->msg_.angular.z = twist_error_.rot(2);
        state_error_publisher_->unlockAndPublish();
      }
    }
    if (state_pose_publisher_){
      if (state_pose_publisher_->trylock()){
	Pose tmp;
        tf::poseKDLToTF(pose_meas_, tmp);
	poseStampedTFToMsg(Stamped<Pose>(tmp, ros::Time::now(), root_name_), state_pose_publisher_->msg_);
        state_pose_publisher_->unlockAndPublish();
      }
    }
  }
}



Frame CartesianPoseController::getPose()
{
  // get the joint positions and velocities
  chain_.getPositions(jnt_pos_);

  // get cartesian pose
  Frame result;
  jnt_to_pose_solver_->JntToCart(jnt_pos_, result);

  return result;
}

void CartesianPoseController::command(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  // convert message to transform
  Stamped<Pose> pose_stamped;
  poseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain
  tf_.transformPose(root_name_, pose_stamped, pose_stamped);
  tf::poseTFToKDL(pose_stamped, pose_desi_);
}

} // namespace

