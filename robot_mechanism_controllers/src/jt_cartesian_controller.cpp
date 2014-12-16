/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

// Author: Stuart Glaser

#include <robot_mechanism_controllers/jt_cartesian_controller.h>

#include <Eigen/LU>

#include <ros/ros.h>

#include <angles/angles.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(controller::JTCartesianController, pr2_controller_interface::Controller)


namespace controller {

JTCartesianController::JTCartesianController()
  : robot_state_(NULL), use_posture_(false)
{}

JTCartesianController::~JTCartesianController()
{
  sub_gains_.shutdown();
  sub_posture_.shutdown();
  sub_pose_.shutdown();
}


bool JTCartesianController::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &n)
{
  node_ = n;

  // get name of root and tip from the parameter server
  std::string tip_name;
  if (!node_.getParam("root_name", root_name_)){
    ROS_ERROR("JTCartesianController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tip_name", tip_name)){
    ROS_ERROR("JTCartesianController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // Chain of joints
  if (!chain_.init(robot_state_, root_name_, tip_name))
    return false;
  if (!chain_.allCalibrated())
  {
    ROS_ERROR("Not all joints in the chain are calibrated (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (chain_.size() != Joints)
  {
    ROS_ERROR("The JTCartesianController works with %d joints, but the chain from %s to %s has %d joints.",
              Joints, root_name_.c_str(), tip_name.c_str(), chain_.size());
    return false;
  }

  // Kinematics
  KDL::Chain kdl_chain;
  chain_.toKDL(kdl_chain);
  kin_.reset(new Kin<Joints>(kdl_chain));

  // Cartesian gains
  double kp_trans, kd_trans, kp_rot, kd_rot;
  if (!node_.getParam("cart_gains/trans/p", kp_trans) ||
      !node_.getParam("cart_gains/trans/d", kd_trans))
  {
    ROS_ERROR("P and D translational gains not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("cart_gains/rot/p", kp_rot) ||
      !node_.getParam("cart_gains/rot/d", kd_rot))
  {
    ROS_ERROR("P and D rotational gains not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  Kp << kp_trans, kp_trans, kp_trans,  kp_rot, kp_rot, kp_rot;
  Kd << kd_trans, kd_trans, kd_trans,  kd_rot, kd_rot, kd_rot;

  node_.param("pose_command_filter", pose_command_filter_, 1.0);

  // Velocity saturation
  node_.param("vel_saturation_trans", vel_saturation_trans_, 0.0);
  node_.param("vel_saturation_rot", vel_saturation_rot_, 0.0);

  node_.param("jacobian_inverse_damping", jacobian_inverse_damping_, 0.0);
  node_.param("joint_vel_filter", joint_vel_filter_, 1.0);

  // Joint gains
  for (int i = 0; i < Joints; ++i)
    node_.param("joint_feedforward/" + chain_.getJoint(i)->joint_->name, joint_dd_ff_[i], 0.0);
  for (int i = 0; i < Joints; ++i)
    node_.param("saturation/" + chain_.getJoint(i)->joint_->name, saturation_[i], 0.0);

  // Posture gains
  node_.param("k_posture", k_posture_, 1.0);

  node_.param("resolution/force", res_force_, 0.01);
  node_.param("resolution/position", res_position_, 0.001);
  node_.param("resolution/torque", res_torque_, 0.01);
  node_.param("resolution/orientation", res_orientation_, 0.001);


  sub_gains_ = node_.subscribe("gains", 5, &JTCartesianController::setGains, this);
  sub_posture_ = node_.subscribe("command_posture", 5, &JTCartesianController::commandPosture, this);
  sub_pose_ = node_.subscribe("command_pose", 1, &JTCartesianController::commandPose, this);

  StateMsg state_template;
  state_template.header.frame_id = root_name_;
  state_template.x.header.frame_id = root_name_;
  state_template.x_desi.header.frame_id = root_name_;
  state_template.x_desi_filtered.header.frame_id = root_name_;
  state_template.tau_pose.resize(Joints);
  state_template.tau_posture.resize(Joints);
  state_template.tau.resize(Joints);
  state_template.J.layout.dim.resize(2);
  state_template.J.data.resize(6*Joints);
  state_template.N.layout.dim.resize(2);
  state_template.N.data.resize(Joints*Joints);
  pub_state_.init(node_, "state", 10);
  pub_state_.lock();
  pub_state_.msg_ = state_template;
  pub_state_.unlock();

  pub_x_desi_.init(node_, "state/x_desi", 10);
  pub_x_desi_.lock();
  pub_x_desi_.msg_.header.frame_id = root_name_;
  pub_x_desi_.unlock();

  return true;
}

void JTCartesianController::starting()
{
  //Kp << 800.0, 800.0, 800.0,   80.0, 80.0, 80.0;
  //Kd << 12.0, 12.0, 12.0,   0.0, 0.0, 0.0;

  JointVec q;
  chain_.getPositions(q);
  kin_->fk(q, x_desi_);
  x_desi_filtered_ = x_desi_;
  last_pose_ = x_desi_;
  q_posture_ = q;
  qdot_filtered_.setZero();
  last_wrench_.setZero();

  loop_count_ = 0;
}


static void computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d &xdes, Eigen::Matrix<double,6,1> &err)
{
  err.head<3>() = xact.translation() - xdes.translation();
  err.tail<3>()   = 0.5 * (xdes.linear().col(0).cross(xact.linear().col(0)) +
                          xdes.linear().col(1).cross(xact.linear().col(1)) +
                          xdes.linear().col(2).cross(xact.linear().col(2)));
}

void JTCartesianController::update()
{
  // get time
  ros::Time time = robot_state_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;
  ++loop_count_;

  // ======== Measures current arm state

  JointVec q;
  chain_.getPositions(q);

  Eigen::Affine3d x;
  kin_->fk(q, x);

  Jacobian J;
  kin_->jac(q, J);


  JointVec qdot_raw;
  chain_.getVelocities(qdot_raw);
  for (int i = 0; i < Joints; ++i)
    qdot_filtered_[i] += joint_vel_filter_ * (qdot_raw[i] - qdot_filtered_[i]);
  JointVec qdot = qdot_filtered_;
  CartVec xdot = J * qdot;

  // ======== Controls to the current pose setpoint

  {
    Eigen::Vector3d p0(x_desi_filtered_.translation());
    Eigen::Vector3d p1(x_desi_.translation());
    Eigen::Quaterniond q0(x_desi_filtered_.linear());
    Eigen::Quaterniond q1(x_desi_.linear());
    q0.normalize();
    q1.normalize();

    tf::Quaternion tf_q0(q0.x(), q0.y(), q0.z(), q0.w());
    tf::Quaternion tf_q1(q1.x(), q1.y(), q1.z(), q1.w());
    tf::Quaternion tf_q = tf_q0.slerp(tf_q1, pose_command_filter_);

    Eigen::Vector3d p = p0 + pose_command_filter_ * (p1 - p0);
    //Eigen::Quaterniond q = q0.slerp(pose_command_filter_, q1);
    Eigen::Quaterniond q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
    //x_desi_filtered_ = q * Eigen::Translation3d(p);
    x_desi_filtered_ = Eigen::Translation3d(p) * q;
  }
  CartVec x_err;
  //computePoseError(x, x_desi_, x_err);
  computePoseError(x, x_desi_filtered_, x_err);

  CartVec xdot_desi = (Kp.array() / Kd.array()) * x_err.array() * -1.0;

  // Caps the cartesian velocity
  if (vel_saturation_trans_ > 0.0)
  {
    if (fabs(xdot_desi.head<3>().norm()) > vel_saturation_trans_)
      xdot_desi.head<3>() *= (vel_saturation_trans_ / xdot_desi.head<3>().norm());
  }
  if (vel_saturation_rot_ > 0.0)
  {
    if (fabs(xdot_desi.tail<3>().norm()) > vel_saturation_rot_)
      xdot_desi.tail<3>() *= (vel_saturation_rot_ / xdot_desi.tail<3>().norm());
  }

  CartVec F = Kd.array() * (xdot_desi - xdot).array();

  JointVec tau_pose = J.transpose() * F;

  // ======== J psuedo-inverse and Nullspace computation

  // Computes pseudo-inverse of J
  Eigen::Matrix<double,6,6> I6; I6.setIdentity();
  //Eigen::Matrix<double,6,6> JJt = J * J.transpose();
  //Eigen::Matrix<double,6,6> JJt_inv = JJt.inverse();
  Eigen::Matrix<double,6,6> JJt_damped = J * J.transpose() + jacobian_inverse_damping_ * I6;
  Eigen::Matrix<double,6,6> JJt_inv_damped = JJt_damped.inverse();
  Eigen::Matrix<double,Joints,6> J_pinv = J.transpose() * JJt_inv_damped;

  // Computes the nullspace of J
  Eigen::Matrix<double,Joints,Joints> I;
  I.setIdentity();
  Eigen::Matrix<double,Joints,Joints> N = I - J_pinv * J;

  // ======== Posture control

  // Computes the desired joint torques for achieving the posture
  JointVec tau_posture;
  tau_posture.setZero();
  if (use_posture_)
  {
    JointVec posture_err = q_posture_ - q;
    for (size_t j = 0; j < Joints; ++j)
    {
      if (chain_.getJoint(j)->joint_->type == urdf::Joint::CONTINUOUS)
        posture_err[j] = angles::normalize_angle(posture_err[j]);
    }

    for (size_t j = 0; j < Joints; ++j) {
      if (fabs(q_posture_[j] - 9999) < 1e-5)
        posture_err[j] = 0.0;
    }

    JointVec qdd_posture = k_posture_ * posture_err;
    tau_posture = joint_dd_ff_.array() * (N * qdd_posture).array();
  }

  JointVec tau = tau_pose + tau_posture;

  // ======== Torque Saturation
  double sat_scaling = 1.0;
  for (int i = 0; i < Joints; ++i) {
    if (saturation_[i] > 0.0)
      sat_scaling = std::min(sat_scaling, fabs(saturation_[i] / tau[i]));
  }
  JointVec tau_sat = sat_scaling * tau;

  chain_.addEfforts(tau_sat);

  if (loop_count_ % 10 == 0)
  {
    if (pub_x_desi_.trylock()) {
      pub_x_desi_.msg_.header.stamp = time;
      tf::poseEigenToMsg(x_desi_, pub_x_desi_.msg_.pose);
      pub_x_desi_.unlockAndPublish();
    }

    if (pub_state_.trylock()) {
      pub_state_.msg_.header.stamp = time;
      pub_state_.msg_.x.header.stamp = time;
      tf::poseEigenToMsg(x, pub_state_.msg_.x.pose);
      pub_state_.msg_.x_desi.header.stamp = time;
      tf::poseEigenToMsg(x_desi_, pub_state_.msg_.x_desi.pose);
      pub_state_.msg_.x_desi_filtered.header.stamp = time;
      tf::poseEigenToMsg(x_desi_filtered_, pub_state_.msg_.x_desi_filtered.pose);
      tf::twistEigenToMsg(x_err, pub_state_.msg_.x_err);
      tf::twistEigenToMsg(xdot, pub_state_.msg_.xd);
      tf::twistEigenToMsg(xdot_desi, pub_state_.msg_.xd_desi);
      tf::wrenchEigenToMsg(F, pub_state_.msg_.F);
      tf::matrixEigenToMsg(J, pub_state_.msg_.J);
      tf::matrixEigenToMsg(N, pub_state_.msg_.N);
      for (size_t j = 0; j < Joints; ++j) {
        pub_state_.msg_.tau_pose[j] = tau_pose[j];
        pub_state_.msg_.tau_posture[j] = tau_posture[j];
        pub_state_.msg_.tau[j] = tau[j];
      }
      pub_state_.unlockAndPublish();
    }
  }
}

} //namespace
