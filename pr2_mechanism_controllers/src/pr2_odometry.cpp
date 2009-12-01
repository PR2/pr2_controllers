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
/*
 * Author: Sachin Chitta and Matthew Piccoli
 */

#include "pr2_mechanism_controllers/pr2_odometry.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(Pr2Odometry, controller::Pr2Odometry, pr2_controller_interface::Controller)

namespace controller {

Pr2Odometry::Pr2Odometry()
{
}

Pr2Odometry::~Pr2Odometry()
{
}

bool Pr2Odometry::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node)
{
  node_ = node;
  node.param("odometer/distance", odometer_distance_, 0.0);
  node.param("odometer/angle", odometer_angle_, 0.0);
  node.param("odom/x", odom_.x, 0.0);
  node.param("odom/y", odom_.y, 0.0);
  node.param("odom/z", odom_.z, 0.0);

  node.param<std::string> ("ils_weight_type", ils_weight_type_, "Gaussian");
  node.param<int> ("ils_max_iterations", ils_max_iterations_, 3);
  node.param<std::string> ("odom_frame", odom_frame_, "odom");
  node.param<std::string> ("base_footprint_frame", base_footprint_frame_, "base_footprint");
  node.param<std::string> ("base_link_frame", base_link_frame_, "base_link");
  node.param<double> ("base_link_floor_z_offset",  base_link_floor_z_offset_, 0.051);

  node.param<double> ("sigma_x", sigma_x_, 0.002);
  node.param<double> ("sigma_y", sigma_y_, 0.002);
  node.param<double> ("sigma_theta", sigma_theta_, 0.017);

  node.param<double> ("cov_x_y", cov_x_y_, 0.0);
  node.param<double> ("cov_x_theta", cov_x_theta_, 0.0);
  node.param<double> ("cov_y_theta", cov_y_theta_, 0.0);


  node.param("expected_publish_time", expected_publish_time_, 0.03);

  if(!base_kin_.init(robot_state, node_))
    return false;

  cbv_lhs_ = Eigen::MatrixXf::Zero(16, 3);
  cbv_rhs_ = Eigen::MatrixXf::Zero(16, 1);
  cbv_soln_ = Eigen::MatrixXf::Zero(3, 1);

  fit_lhs_ = Eigen::MatrixXf::Zero(16, 3);
  fit_rhs_ = Eigen::MatrixXf::Zero(16, 1);
  fit_soln_ = Eigen::MatrixXf::Zero(3, 1);

  fit_residual_ = Eigen::MatrixXf::Zero(16, 1);
  odometry_residual_ = Eigen::MatrixXf::Zero(16, 1);

  weight_matrix_ = Eigen::MatrixXf::Identity(16, 16);

  odometry_publisher_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(node_,odom_frame_, 1));
  transform_publisher_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(node_,"/tf", 1));
  transform_publisher_->msg_.set_transforms_size(2);

  return true;
}

bool Pr2Odometry::initXml(pr2_mechanism_model::RobotState *robot_state, TiXmlElement *config)
{
  ros::NodeHandle n(config->Attribute("name"));
  return init(robot_state, n);
}

void Pr2Odometry::starting()
{
  current_time_ = base_kin_.robot_state_->getTime();
  last_time_ = base_kin_.robot_state_->getTime();
  last_publish_time_ = base_kin_.robot_state_->getTime();
}

void Pr2Odometry::update()
{
  current_time_ = base_kin_.robot_state_->getTime();
  updateOdometry();
  publish();
  last_time_ = current_time_;
}

void Pr2Odometry::updateOdometry()
{
  double dt = (current_time_ - last_time_).toSec();
  double theta = odom_.z;
  double costh = cos(theta);
  double sinth = sin(theta);

  computeBaseVelocity();

  double odom_delta_x = (odom_vel_.linear.x * costh - odom_vel_.linear.y * sinth) * dt;
  double odom_delta_y = (odom_vel_.linear.x * sinth + odom_vel_.linear.y * costh) * dt;
  double odom_delta_th = odom_vel_.angular.z * dt;

  odom_.x += odom_delta_x;
  odom_.y += odom_delta_y;
  odom_.z += odom_delta_th;

  odometer_distance_ += sqrt(odom_delta_x * odom_delta_x + odom_delta_y * odom_delta_y);
  odometer_angle_ += fabs(odom_delta_th);
}

void Pr2Odometry::getOdometry(geometry_msgs::Point &odom, geometry_msgs::Twist &odom_vel)
{
  odom = odom_;
  odom_vel = odom_vel_;
  return;
}

void Pr2Odometry::getOdometryMessage(nav_msgs::Odometry &msg)
{
  msg.header.frame_id = odom_frame_;
  msg.header.stamp = current_time_;
  msg.pose.pose.position.x = odom_.x;
  msg.pose.pose.position.y = odom_.y;
  msg.pose.pose.position.z = 0.0;

  tf::Quaternion quat_trans = tf::Quaternion(odom_.z, 0.0, 0.0);
  msg.pose.pose.orientation.x = quat_trans.x();
  msg.pose.pose.orientation.y = quat_trans.y();
  msg.pose.pose.orientation.z = quat_trans.z();
  msg.pose.pose.orientation.w = quat_trans.w();

  msg.twist.twist = odom_vel_;
/*  msg.twist.linear.x = odom_vel_.x*cos(odom_.z)-odom_vel_.y*sin(odom_.z);
  msg.twist.linear.y = odom_vel_.x*sin(odom_.z)+odom_vel_.y*cos(odom_.z);
*/
  populateCovariance(odometry_residual_max_,msg);
}

void Pr2Odometry::populateCovariance(double residual, nav_msgs::Odometry &msg)
{
  // multiplier to scale covariance
  // the smaller the residual, the more reliable odom
  double odom_multiplier;
  if (residual < 0.05)
  {
    odom_multiplier = ((residual-0.00001)*(0.99999/0.04999))+0.00001;
  }
  else
  {
    odom_multiplier = ((residual-0.05)*(19.0/0.15))+1.0;
  }
  odom_multiplier = fmax(0.00001, fmin(100.0, odom_multiplier));
  odom_multiplier *= 2.0;

  //nav_msgs::Odometry has a 6x6 covariance matrix
  msg.pose.covariance[0] = odom_multiplier*pow(sigma_x_,2);
  msg.pose.covariance[7] = odom_multiplier*pow(sigma_y_,2);
  msg.pose.covariance[35] = odom_multiplier*pow(sigma_theta_,2);

  msg.pose.covariance[1] = odom_multiplier*cov_x_y_;
  msg.pose.covariance[6] = odom_multiplier*cov_x_y_;

  msg.pose.covariance[31] = odom_multiplier*cov_y_theta_;
  msg.pose.covariance[11] = odom_multiplier*cov_y_theta_;

  msg.pose.covariance[30] = odom_multiplier*cov_x_theta_;
  msg.pose.covariance[5] =  odom_multiplier*cov_x_theta_;

  msg.pose.covariance[14] = DBL_MAX;
  msg.pose.covariance[21] = DBL_MAX;
  msg.pose.covariance[28] = DBL_MAX;

  msg.twist.covariance = msg.pose.covariance;
}


void Pr2Odometry::getOdometry(double &x, double &y, double &yaw, double &vx, double &vy, double &vw)
{
  x = odom_.x;
  y = odom_.y;
  yaw = odom_.z;
  vx = odom_vel_.linear.x;
  vy = odom_vel_.linear.y;
  vw = odom_vel_.angular.z;
}

void Pr2Odometry::computeBaseVelocity()
{
  double steer_angle, wheel_speed, costh, sinth;
  geometry_msgs::Twist caster_local_velocity;
  geometry_msgs::Twist wheel_local_velocity;
  geometry_msgs::Point wheel_position;
  for(int i = 0; i < base_kin_.num_wheels_; i++)
  {
    base_kin_.wheel_[i].updatePosition();
    steer_angle = base_kin_.wheel_[i].parent_->joint_->position_;//I'm coming out properly!
    wheel_position = base_kin_.wheel_[i].position_;
    costh = cos(steer_angle);
    sinth = sin(steer_angle);
    wheel_speed = getCorrectedWheelSpeed(i);
    cbv_rhs_(i * 2, 0) = base_kin_.wheel_radius_ * base_kin_.wheel_[i].wheel_radius_scaler_ * wheel_speed;
    cbv_rhs_(i * 2 + 1, 0) = 0;

    cbv_lhs_(i * 2, 0) = costh;
    cbv_lhs_(i * 2, 1) = sinth;
    cbv_lhs_(i * 2, 2) = -costh * wheel_position.y + sinth * wheel_position.x;
    cbv_lhs_(i * 2 + 1, 0) = -sinth;
    cbv_lhs_(i * 2 + 1, 1) = costh;
    cbv_lhs_(i * 2 + 1, 2) = sinth * wheel_position.y + costh * wheel_position.x;
  }
  cbv_soln_ = iterativeLeastSquares(cbv_lhs_, cbv_rhs_, ils_weight_type_, ils_max_iterations_);

  odometry_residual_ = cbv_lhs_ * cbv_soln_ - cbv_rhs_;
  odometry_residual_max_ = odometry_residual_.cwise().abs().maxCoeff();

  odom_vel_.linear.x = cbv_soln_(0, 0);
  odom_vel_.linear.y = cbv_soln_(1, 0);
  odom_vel_.angular.z = cbv_soln_(2, 0);
}

double Pr2Odometry::getCorrectedWheelSpeed(int index)
{
  double wheel_speed;
  geometry_msgs::Twist caster_local_vel;
  geometry_msgs::Twist wheel_local_vel;
  caster_local_vel.angular.z = base_kin_.wheel_[index].parent_->joint_->velocity_;
  wheel_local_vel = base_kin_.pointVel2D(base_kin_.wheel_[index].offset_, caster_local_vel);
  wheel_speed = base_kin_.wheel_[index].joint_->velocity_ - wheel_local_vel.linear.x / (base_kin_.wheel_radius_ * base_kin_.wheel_[index].wheel_radius_scaler_);
  return wheel_speed;
}

Eigen::MatrixXf Pr2Odometry::iterativeLeastSquares(Eigen::MatrixXf lhs, Eigen::MatrixXf rhs, std::string weight_type, int max_iter)
{
  weight_matrix_ = Eigen::MatrixXf::Identity(16, 16);
  for(int i = 0; i < max_iter; i++)
  {
    fit_lhs_ = weight_matrix_ * lhs;
    fit_rhs_ = weight_matrix_ * rhs;

    fit_lhs_.svd().solve(fit_rhs_, &fit_soln_);
    fit_residual_ = rhs - lhs * fit_soln_;

    for(int j = 0; j < base_kin_.num_wheels_; j++)
    {
      int fw = 2 * j;
      int sw = fw + 1;
      if(fabs(fit_residual_(fw, 0)) > fabs(fit_residual_(sw, 0)))
      {
        fit_residual_(fw, 0) = fabs(fit_residual_(fw, 0));
        fit_residual_(sw, 0) = fit_residual_(fw, 0);
      }
      else
      {
        fit_residual_(fw, 0) = fabs(fit_residual_(sw, 0));
        fit_residual_(sw, 0) = fit_residual_(fw, 0);
      }
    }
    weight_matrix_ = findWeightMatrix(fit_residual_, weight_type);
  }
  return fit_soln_;
}

Eigen::MatrixXf Pr2Odometry::findWeightMatrix(Eigen::MatrixXf residual, std::string weight_type)
{
  Eigen::MatrixXf w_fit = Eigen::MatrixXf::Identity(16, 16);
  double epsilon = 0;
  double g_sigma = 0.1;

  if(weight_type == std::string("BubeLagan"))
  {
    for(int i = 0; i < 2 * base_kin_.num_wheels_; i++)
    {
      if(fabs(residual(i, 0) > epsilon))
        epsilon = fabs(residual(i, 0));
    }
    epsilon = epsilon / 100.0;
  }
  for(int i = 0; i < 2 * base_kin_.num_wheels_; i++)
  {
    if(weight_type == std::string("L1norm"))
    {
      w_fit(i, i) = 1.0 / (1 + sqrt(fabs(residual(i, 0))));
    }
    else if(weight_type == std::string("fair"))
    {
      w_fit(i, i) = 1.0 / (1 + fabs(residual(i, 0)));
    }
    else if(weight_type == std::string("Cauchy"))
    {
      w_fit(i, i) = 1.0 / (1 + residual(i, 0) * residual(i, 0));
    }
    else if(weight_type == std::string("BubeLangan"))
    {
      w_fit(i, i) = 1.0 / pow((1 + pow((residual(i, 0) / epsilon), 2)), 0.25);
    }
    else if(weight_type == std::string("Gaussian"))
    {
      w_fit(i, i) = sqrt(exp(-pow(residual(i, 0), 2) / (2 * g_sigma * g_sigma)));
    }
    else // default to fair
    {
      w_fit(i, i) = 1.0 / (0.1 + sqrt(fabs(residual(i, 0))));
    }
  }
  return w_fit;
}

void Pr2Odometry::publish()
{
  if(fabs((last_publish_time_ - current_time_).toSec()) < expected_publish_time_)
    return;

  if(odometry_publisher_->trylock())
  {
    getOdometryMessage(odometry_publisher_->msg_);
    odometry_publisher_->unlockAndPublish();
  }

  if(transform_publisher_->trylock())
  {

    double x(0.), y(0.0), yaw(0.0), vx(0.0), vy(0.0), vyaw(0.0);
    this->getOdometry(x, y, yaw, vx, vy, vyaw);

    geometry_msgs::TransformStamped &out = transform_publisher_->msg_.transforms[0];
    out.header.stamp = current_time_;
    out.header.frame_id =  base_footprint_frame_;
    out.child_frame_id = odom_frame_;
    out.transform.translation.x = -x * cos(yaw) - y * sin(yaw);
    out.transform.translation.y = +x * sin(yaw) - y * cos(yaw);
    out.transform.translation.z = 0;
    tf::Quaternion quat_trans = tf::Quaternion(-yaw, 0.0, 0.0);

    out.transform.rotation.x = quat_trans.x();
    out.transform.rotation.y = quat_trans.y();
    out.transform.rotation.z = quat_trans.z();
    out.transform.rotation.w = quat_trans.w();

    geometry_msgs::TransformStamped &out2 = transform_publisher_->msg_.transforms[1];
    out2.header.stamp = current_time_;
    out2.header.frame_id = base_footprint_frame_;
    out2.child_frame_id = base_link_frame_;
    out2.transform.translation.x = 0;
    out2.transform.translation.y = 0;

    // FIXME: this is the offset between base_link origin and the ideal floor
    out2.transform.translation.z = base_link_floor_z_offset_; // FIXME: this is hardcoded, considering we are deprecating base_footprint soon, I will not get this from URDF.

    out2.transform.rotation.x = 0;
    out2.transform.rotation.y = 0;
    out2.transform.rotation.z = 0;
    out2.transform.rotation.w = 1;

    transform_publisher_->unlockAndPublish();
  }

  last_publish_time_ = current_time_;

}
} // namespace
