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



#include "robot_mechanism_controllers/cartesian_pose_controller.h"
#include <algorithm>
#include "kdl/chainfksolverpos_recursive.hpp"
#include "pluginlib/class_list_macros.h"


using namespace KDL;
using namespace tf;
using namespace std;

PLUGINLIB_REGISTER_CLASS(CartesianPoseController, controller::CartesianPoseController, pr2_controller_interface::Controller)

namespace controller {

CartesianPoseController::CartesianPoseController()
: robot_state_(NULL),
  jnt_to_pose_solver_(NULL),
  state_error_publisher_(NULL),
  state_pose_publisher_(NULL),
  command_notifier_(NULL)
{}

CartesianPoseController::~CartesianPoseController()
{}


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
  chain_.toKDL(kdl_chain_);

  // create solver
  jnt_to_pose_solver_.reset(new ChainFkSolverPos_recursive(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());

  // create 6 identical pid controllers for x, y and z translation, and x, y, z rotation
  control_toolbox::Pid pid_controller;
  if (!pid_controller.init(node_)) return false;
  for (unsigned int i=0; i<6; i++)
    pid_controller_.push_back(pid_controller);

  // get a pointer to the twist controller
  string output;
  if (!node_.getParam("output", output)){
    ROS_ERROR("No ouptut name found on parameter server (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!getController<CartesianTwistController>(output, AFTER_ME, twist_controller_)){
    ROS_ERROR("Could not connect to twist controller \"%s\"", output.c_str());
    return false;
  }

  // subscribe to pose commands
  command_notifier_.reset(new MessageNotifier<geometry_msgs::PoseStamped>(tf_,
                                                                       boost::bind(&CartesianPoseController::command, this, _1),
                                                                       node_.getNamespace() + "/command", root_name_, 10));
  // realtime publisher for control state
  state_error_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(node_, "state/error", 1));
  state_pose_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node_, "state/pose", 1));

  return true;
}

bool CartesianPoseController::starting()
{
  // reset pid controllers
  for (unsigned int i=0; i<6; i++)
    pid_controller_[i].reset();

  // initialize desired pose/twist
  twist_ff_ = Twist::Zero();
  pose_desi_ = getPose();
  last_time_ = robot_state_->getTime();

  loop_count_ = 0;
  return true;
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
  twist_error_ = diff(pose_desi_, pose_meas_);
  Twist twist_fb;
  for (unsigned int i=0; i<6; i++)
    twist_fb(i) = pid_controller_[i].updatePid(twist_error_(i), dt);

  // send feedback twist and feedforward twist to twist controller
  twist_controller_->twist_desi_ = twist_fb + twist_ff_;

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
	frameToPose(pose_meas_, tmp);
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

void CartesianPoseController::command(const tf::MessageNotifier<geometry_msgs::PoseStamped>::MessagePtr& pose_msg)
{
  // convert message to transform
  Stamped<Pose> pose_stamped;
  poseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain
  tf_.transformPose(root_name_, pose_stamped, pose_stamped);
  poseToFrame(pose_stamped, pose_desi_);
}


void CartesianPoseController::poseToFrame(const Pose& pose, Frame& frame)
{
  frame.p(0) = pose.getOrigin().x();
  frame.p(1) = pose.getOrigin().y();
  frame.p(2) = pose.getOrigin().z();

  double Rz, Ry, Rx;
  pose.getBasis().getEulerZYX(Rz, Ry, Rx);
  frame.M = Rotation::EulerZYX(Rz, Ry, Rx);
}


void CartesianPoseController::frameToPose(const Frame& frame, Pose& pose)
{
  pose.getOrigin()[0] = frame.p(0);
  pose.getOrigin()[1] = frame.p(1);
  pose.getOrigin()[2] = frame.p(2);

  double Rz, Ry, Rx;
  frame.M.GetEulerZYX(Rz, Ry, Rx);
  pose.setRotation( Quaternion(Rz, Ry, Rx));
}




} // namespace

