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

#ifndef PR2_CONTROLLERS_JT_CARTESIAN_CONTROLLER_H
#define PR2_CONTROLLERS_JT_CARTESIAN_CONTROLLER_H

#include <boost/scoped_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <control_toolbox/pid.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <robot_mechanism_controllers/JTCartesianControllerState.h>


namespace controller {

template <int Joints>
struct Kin
{
  typedef Eigen::Matrix<double, Joints, 1> JointVec;
  typedef Eigen::Matrix<double, 6, Joints> Jacobian;

  Kin(const KDL::Chain &kdl_chain) :
    fk_solver_(kdl_chain), jac_solver_(kdl_chain),
    kdl_q(Joints), kdl_J(Joints)
  {
  }
  ~Kin()
  {
  }

  void fk(const JointVec &q, Eigen::Affine3d &x)
  {
    kdl_q.data = q;
    KDL::Frame kdl_x;
    fk_solver_.JntToCart(kdl_q, kdl_x);
    tf::transformKDLToEigen(kdl_x, x);
  }
  void jac(const JointVec &q, Jacobian &J)
  {
    kdl_q.data = q;
    jac_solver_.JntToJac(kdl_q, kdl_J);
    J = kdl_J.data;
  }

  KDL::ChainFkSolverPos_recursive fk_solver_;
  KDL::ChainJntToJacSolver jac_solver_;
  KDL::JntArray kdl_q;
  KDL::Jacobian kdl_J;
};

class JTCartesianController : public pr2_controller_interface::Controller
{
public:
  // Ensure 128-bit alignment for Eigen
  // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
  enum { Joints = 7 };
  typedef Eigen::Matrix<double, Joints, 1> JointVec;
  typedef Eigen::Matrix<double, 6, 1> CartVec;
  typedef Eigen::Matrix<double, 6, Joints> Jacobian;
  typedef robot_mechanism_controllers::JTCartesianControllerState StateMsg;
public:
  JTCartesianController();
  ~JTCartesianController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  void starting();
  void update();

  Eigen::Affine3d x_desi_, x_desi_filtered_;
  CartVec wrench_desi_;

  ros::NodeHandle node_;
  ros::Subscriber sub_gains_;
  ros::Subscriber sub_posture_;
  ros::Subscriber sub_pose_;
  tf::TransformListener tf_;

  realtime_tools::RealtimePublisher<StateMsg> pub_state_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> pub_x_desi_;

  std::string root_name_, tip_name_;
  ros::Time last_time_;
  int loop_count_;
  pr2_mechanism_model::RobotState *robot_state_;

  pr2_mechanism_model::Chain chain_;
  boost::scoped_ptr<Kin<Joints> > kin_;
  Eigen::Matrix<double,6,1> Kp, Kd;
  double pose_command_filter_;
  double vel_saturation_trans_, vel_saturation_rot_;
  JointVec saturation_;
  JointVec joint_dd_ff_;
  double joint_vel_filter_;
  double jacobian_inverse_damping_;
  JointVec q_posture_;
  double k_posture_;
  bool use_posture_;

  // Minimum resolutions
  double res_force_, res_position_;
  double res_torque_, res_orientation_;

  Eigen::Affine3d last_pose_;
  CartVec last_wrench_;

  JointVec qdot_filtered_;

  void setGains(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    if (msg->data.size() >= 6)
      for (size_t i = 0; i < 6; ++i)
        Kp[i] = msg->data[i];
    if (msg->data.size() == 12)
      for (size_t i = 0; i < 6; ++i)
        Kd[i] = msg->data[6+i];

    ROS_INFO("New gains: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf",
             Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
  }

  void commandPosture(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    if (msg->data.size() == 0) {
      use_posture_ = false;
      ROS_INFO("Posture turned off");
    }
    else if ((int)msg->data.size() != q_posture_.size()) {
      ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
      return;
    }
    else
    {
      use_posture_ = true;
      for (int j = 0; j < Joints; ++j)
        q_posture_[j] = msg->data[j];
    }
  }

  void commandPose(const geometry_msgs::PoseStamped::ConstPtr &command)
  {
    geometry_msgs::PoseStamped in_root;
    try {
      tf_.waitForTransform(root_name_, command->header.frame_id, command->header.stamp, ros::Duration(0.1));
      tf_.transformPose(root_name_, *command, in_root);
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR("Failed to transform: %s", ex.what());
      return;
    }

    tf::poseMsgToEigen(in_root.pose, x_desi_);
  }
};

} // namespace

#endif
