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

PLUGINLIB_EXPORT_CLASS( controller::Pr2Odometry, pr2_controller_interface::Controller)

namespace controller {


  const static double ODOMETRY_THRESHOLD = 1e-4;
  const static double MAX_ALLOWABLE_SVD_TIME = 3e-4;

  Pr2Odometry::Pr2Odometry()
  {
    sequence_ = 0;
  }

  Pr2Odometry::~Pr2Odometry()
  {
  }

  bool Pr2Odometry::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node)
  {
    node_ = node;

    std::string prefix_param;
    node.searchParam("tf_prefix", prefix_param);
    node.getParam(prefix_param, tf_prefix_);

    node.param<double>("odometer/initial_distance", odometer_distance_, 0.0);
    node.param<double>("odometer/initial_angle", odometer_angle_, 0.0);
    node.param<double>("odom/initial_x", odom_.x, 0.0);
    node.param<double>("odom/initial_y", odom_.y, 0.0);
    node.param<double>("odom/initial_yaw", odom_.z, 0.0);

    node.param<bool>("publish_tf", publish_tf_, true);
    node.param<int> ("ils_max_iterations", ils_max_iterations_, 3);
    node.param<std::string> ("odom_frame", odom_frame_, "odom");
    node.param<std::string> ("base_footprint_frame", base_footprint_frame_,
        "base_footprint");
    node.param<std::string> ("base_link_frame", base_link_frame_, "base_link");

    node.param<double> ("x_stddev", sigma_x_, 0.002);
    node.param<double> ("y_stddev", sigma_y_, 0.002);
    node.param<double> ("rotation_stddev", sigma_theta_, 0.017);

    node.param<double> ("cov_xy", cov_x_y_, 0.0);
    node.param<double> ("cov_xrotation", cov_x_theta_, 0.0);
    node.param<double> ("cov_yrotation", cov_y_theta_, 0.0);
    node.param<bool> ("verbose", verbose_, false);

    node.param<double>("odom_publish_rate", odom_publish_rate_, 30.0);
    node.param<double>("odometer_publish_rate", odometer_publish_rate_, 1.0);
    node.param<double>("state_publish_rate", state_publish_rate_, 1.0);
    node.param<double>("caster_calibration_multiplier",
        caster_calibration_multiplier_, 1.0);

    if(odom_publish_rate_ <= 0.0) {
      expected_publish_time_ = 0.0;
      publish_odom_ = false;
    } else {
      expected_publish_time_ = 1.0/odom_publish_rate_;
      publish_odom_ = true;
    }

    if(odometer_publish_rate_ <= 0.0) {
      expected_odometer_publish_time_ = 0.0;
      publish_odometer_ = false;
    } else {
      expected_odometer_publish_time_ = 1.0/odometer_publish_rate_;
      publish_odometer_ = true;
    }

    if(state_publish_rate_ <= 0.0) {
      expected_state_publish_time_ = 0.0;
      publish_state_ = false;
    } else {
      expected_state_publish_time_ = 1.0/state_publish_rate_;
      publish_state_ = true;
    }



    if(!base_kin_.init(robot_state, node_))
      return false;

    for(int i = 0; i < base_kin_.num_casters_; ++i) {
      if(!base_kin_.caster_[i].joint_->calibrated_) {
        ROS_ERROR("The Base odometry could not start because the casters were "
           "not calibrated. Relaunch the odometry after you see the caster "
           "calibration finish.");
        return false; // Casters are not calibrated
      }
    }

    cbv_rhs_.setZero();
    cbv_lhs_.setZero();
    cbv_soln_.setZero();
    fit_lhs_.setZero();
    fit_rhs_.setZero();
    fit_soln_.setZero();
    fit_residual_.setZero();
    odometry_residual_.setZero();
    weight_matrix_.setIdentity();

    if(verbose_) {
      matrix_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_mechanism_controllers::OdometryMatrix>(node_,"odometry_matrix", 1));
      debug_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_mechanism_controllers::DebugInfo>(node_,"debug", 1));
      debug_publisher_->msg_.timing.resize(3);
      matrix_publisher_->msg_.m.resize(48);
    }

    state_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_mechanism_controllers::BaseOdometryState>(node_,"state", 1));
    odometer_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_mechanism_controllers::Odometer>(node_,"odometer", 1));
    odometry_publisher_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(node_,odom_frame_, 1));
    transform_publisher_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(node_,"/tf", 1));

    transform_publisher_->msg_.transforms.resize(1);

    state_publisher_->msg_.wheel_link_names.resize(base_kin_.num_wheels_);
    state_publisher_->msg_.drive_constraint_errors.resize(base_kin_.num_wheels_);
    state_publisher_->msg_.longitudinal_slip_constraint_errors.resize(base_kin_.num_wheels_);
    return true;
  }

  bool Pr2Odometry::isInputValid()
  {
    return true;

    for(int i=0; i < base_kin_.num_wheels_; i++)
      if(std::isnan(base_kin_.wheel_[i].joint_->velocity_) || 
          std::isnan(base_kin_.wheel_[i].joint_->velocity_))
        return false;

    for(int i=0; i < base_kin_.num_casters_; i++)
      if(std::isnan(base_kin_.caster_[i].joint_->velocity_) || 
          std::isnan(base_kin_.caster_[i].joint_->velocity_))
        return false;

    return true;
  }

  void Pr2Odometry::starting()
  {
    current_time_ = base_kin_.robot_state_->getTime();
    last_time_ = base_kin_.robot_state_->getTime();
    last_publish_time_ = base_kin_.robot_state_->getTime();
    last_transform_publish_time_ = base_kin_.robot_state_->getTime();
    last_state_publish_time_ = base_kin_.robot_state_->getTime();
    last_odometer_publish_time_ = base_kin_.robot_state_->getTime();
  }

  void Pr2Odometry::update()
  {
    if(!isInputValid()) {
      if(verbose_) {
        debug_publisher_->msg_.input_valid = false;
        ROS_DEBUG("Odometry:: Input velocities are invalid");
      }
      return;
    } else {
      if(verbose_)
        debug_publisher_->msg_.input_valid = true;
    }

    current_time_ = base_kin_.robot_state_->getTime();
    ros::Time update_start = ros::Time::now();
    updateOdometry();
    double update_time = (ros::Time::now()-update_start).toSec();
    ros::Time publish_start = ros::Time::now();

    if(publish_odom_)
      publish();
    if(publish_odometer_)
      publishOdometer();
    if(publish_state_)
      publishState();
    if(publish_tf_)
      publishTransform();

    double publish_time = (ros::Time::now()-publish_start).toSec();
    if(verbose_) {
      debug_publisher_->msg_.timing[0] = update_time;
      debug_publisher_->msg_.timing[1] = publish_time;
      debug_publisher_->msg_.residual = odometry_residual_max_;
      debug_publisher_->msg_.sequence = sequence_;
      if(debug_publisher_->trylock()) {
        debug_publisher_->unlockAndPublish();
      }
    }
    last_time_ = current_time_;
    sequence_++;
  }

  void Pr2Odometry::updateOdometry()
  {
    double dt = (current_time_ - last_time_).toSec();
    double theta = odom_.z;
    double costh = cos(theta);
    double sinth = sin(theta);

    computeBaseVelocity();

    double odom_delta_x = (odom_vel_.linear.x * costh - 
                              odom_vel_.linear.y * sinth) * dt;
    double odom_delta_y = (odom_vel_.linear.x * sinth + 
                              odom_vel_.linear.y * costh) * dt;
    double odom_delta_th = odom_vel_.angular.z * dt;

    odom_.x += odom_delta_x;
    odom_.y += odom_delta_y;
    odom_.z += odom_delta_th;

    ROS_DEBUG("Odometry:: Position: %f, %f, %f",odom_.x,odom_.y,odom_.z);

    odometer_distance_ += sqrt(odom_delta_x * odom_delta_x + 
                               odom_delta_y * odom_delta_y);
    odometer_angle_ += fabs(odom_delta_th);
  }

  void Pr2Odometry::getOdometry(geometry_msgs::Point &odom,
                                geometry_msgs::Twist &odom_vel)
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

    tf::Quaternion quat_trans;
    quat_trans.setRPY(0.0, 0.0, odom_.z);
    msg.pose.pose.orientation.x = quat_trans.x();
    msg.pose.pose.orientation.y = quat_trans.y();
    msg.pose.pose.orientation.z = quat_trans.z();
    msg.pose.pose.orientation.w = quat_trans.w();

    msg.twist.twist = odom_vel_;
    populateCovariance(odometry_residual_max_,msg);
  }

  void Pr2Odometry::populateCovariance(const double &residual,
                                       nav_msgs::Odometry &msg)
  {
    double  odom_multiplier = 1.0;

    if(fabs(odom_vel_.linear.x) <= 1e-8 && 
        fabs(odom_vel_.linear.y) <= 1e-8 && 
        fabs(odom_vel_.angular.z) <= 1e-8) {
      //nav_msgs::Odometry has a 6x6 covariance matrix
      msg.pose.covariance[0] = 1e-12;
      msg.pose.covariance[7] = 1e-12;
      msg.pose.covariance[35] = 1e-12;

      msg.pose.covariance[1] = 1e-12;
      msg.pose.covariance[6] = 1e-12;

      msg.pose.covariance[31] = 1e-12;
      msg.pose.covariance[11] = 1e-12;

      msg.pose.covariance[30] = 1e-12;
      msg.pose.covariance[5] =  1e-12;
    } else {
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
    }

    msg.pose.covariance[14] = DBL_MAX;
    msg.pose.covariance[21] = DBL_MAX;
    msg.pose.covariance[28] = DBL_MAX;

    msg.twist.covariance = msg.pose.covariance;
  }


  void Pr2Odometry::getOdometry(double &x, double &y, double &yaw,
                                double &vx, double &vy, double &vw)
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
    for(int i = 0; i < base_kin_.num_wheels_; i++) {
      base_kin_.wheel_[i].updatePosition();
      steer_angle = base_kin_.wheel_[i].parent_->joint_->position_;//I'm coming out properly!
      wheel_position = base_kin_.wheel_[i].position_;
      costh = cos(steer_angle);
      sinth = sin(steer_angle);
      wheel_speed = 0;
      wheel_speed = getCorrectedWheelSpeed(i);
      ROS_DEBUG("Odometry:: Wheel: %s, angle: %f, speed: %f",
          base_kin_.wheel_[i].link_name_.c_str(),steer_angle,wheel_speed);
      cbv_rhs_(i * 2, 0) = base_kin_.wheel_[i].wheel_radius_ * wheel_speed;
      cbv_rhs_(i * 2 + 1, 0) = 0;

      cbv_lhs_(i * 2, 0) = costh;
      cbv_lhs_(i * 2, 1) = sinth;
      cbv_lhs_(i * 2, 2) = -costh * wheel_position.y + 
                            sinth * wheel_position.x;
      cbv_lhs_(i * 2 + 1, 0) = -sinth;
      cbv_lhs_(i * 2 + 1, 1) = costh;
      cbv_lhs_(i * 2 + 1, 2) = sinth * wheel_position.y + 
                               costh * wheel_position.x;
    }
    cbv_soln_ = iterativeLeastSquares(cbv_lhs_, cbv_rhs_, ils_max_iterations_);

    odometry_residual_ = cbv_lhs_ * cbv_soln_ - cbv_rhs_;
    odometry_residual_max_ = odometry_residual_.array().abs().maxCoeff();
    ROS_DEBUG("Odometry:: base velocity: %f, %f, %f",
        cbv_soln_(0,0), cbv_soln_(1,0), cbv_soln_(2,0));
    ROS_DEBUG("Odometry:: odometry residual: %f",odometry_residual_max_);
    odom_vel_.linear.x = cbv_soln_(0, 0);
    odom_vel_.linear.y = cbv_soln_(1, 0);
    odom_vel_.angular.z = cbv_soln_(2, 0);
  }

  double Pr2Odometry::getCorrectedWheelSpeed(const int &index)
  {
    double wheel_speed;
    geometry_msgs::Twist caster_local_vel;
    geometry_msgs::Twist wheel_local_vel;
    caster_local_vel.angular.z = base_kin_.wheel_[index].parent_->joint_->velocity_*caster_calibration_multiplier_;
    wheel_local_vel = base_kin_.pointVel2D(base_kin_.wheel_[index].offset_, caster_local_vel);
    wheel_speed = base_kin_.wheel_[index].joint_->velocity_ - 
      wheel_local_vel.linear.x / (base_kin_.wheel_[index].wheel_radius_);
    return wheel_speed;
  }

  OdomMatrix3x1 Pr2Odometry::iterativeLeastSquares(const OdomMatrix16x3 &lhs,
      const OdomMatrix16x1 &rhs, const int &max_iter)
  {
    weight_matrix_.setIdentity();
    double svd_time = 0.0;
    bool pub_matrix = false;
    ros::Time tmp_start;

    for(int i = 0; i < max_iter; i++) {
      fit_lhs_ = weight_matrix_ * lhs;
      fit_rhs_ = weight_matrix_ * rhs;

      if(verbose_)
        tmp_start = ros::Time::now();
      Eigen::JacobiSVD<Eigen::MatrixXf> svdOfFit(fit_lhs_,Eigen::ComputeThinU|Eigen::ComputeThinV);
      fit_soln_ = svdOfFit.solve(fit_rhs_);

      ROS_DEBUG("Odometry:: fit_soln_: %f %f %f",
          fit_soln_(0,0), fit_soln_(1,0), fit_soln_(2,0));

      if(verbose_) {
        svd_time = (ros::Time::now()-tmp_start).toSec();
        debug_publisher_->msg_.timing[2] = svd_time;
        if(svd_time > MAX_ALLOWABLE_SVD_TIME) {
          for(int k = 0; k < 48; k++) {
            int i_index = k/3;
            int j_index = k - i_index *3;
            matrix_publisher_->msg_.m[i] = fit_lhs_(i_index,j_index);
          }
          pub_matrix = true;
        }
        if(pub_matrix) {
          if(matrix_publisher_->trylock())
            matrix_publisher_->unlockAndPublish();
          break;
        }
      }

      fit_residual_ = rhs - lhs * fit_soln_;
      if(odometry_residual_.array().abs().maxCoeff() <= ODOMETRY_THRESHOLD) {
        ROS_DEBUG("Breaking out since odometry looks good");
        break;
      }
      for(int j = 0; j < base_kin_.num_wheels_; j++) {
        int fw = 2 * j;
        int sw = fw + 1;
        if(fabs(fit_residual_(fw, 0)) > fabs(fit_residual_(sw, 0))) {
          fit_residual_(fw, 0) = fabs(fit_residual_(fw, 0));
          fit_residual_(sw, 0) = fit_residual_(fw, 0);
        } else {
          fit_residual_(fw, 0) = fabs(fit_residual_(sw, 0));
          fit_residual_(sw, 0) = fit_residual_(fw, 0);
        }
      }
      weight_matrix_ = findWeightMatrix(fit_residual_);

    }
    return fit_soln_;
  }

  OdomMatrix16x16 Pr2Odometry::findWeightMatrix(const OdomMatrix16x1 &residual)
  {
    w_fit.setIdentity();
    double g_sigma = 0.1;

    for(int i = 0; i < 2 * base_kin_.num_wheels_; i++) {
      w_fit(i, i) = sqrt(exp(-pow(residual(i, 0), 2) / 
                              (2 * g_sigma * g_sigma)   ));
    }
    return w_fit;
  }

  void Pr2Odometry::publishOdometer()
  {
    if(fabs((last_odometer_publish_time_ - current_time_).toSec()) < 
        expected_odometer_publish_time_)
      return;
    if(odometer_publisher_->trylock()) {
      odometer_publisher_->msg_.distance = odometer_distance_;
      odometer_publisher_->msg_.angle = odometer_angle_;
      odometer_publisher_->unlockAndPublish();
      last_odometer_publish_time_ = current_time_;
    }
  }

  void Pr2Odometry::publishState()
  {
    if(fabs((last_state_publish_time_ - current_time_).toSec()) < 
        expected_state_publish_time_)
      return;
    if(state_publisher_->trylock()) {
      for(int i=0; i < base_kin_.num_wheels_; i++) {
        state_publisher_->msg_.wheel_link_names[i] = 
          base_kin_.wheel_[i].link_name_;

        state_publisher_->msg_.drive_constraint_errors[i] = 
          odometry_residual_(2*i,0);

        state_publisher_->msg_.longitudinal_slip_constraint_errors[i] = 
          odometry_residual_(2*i+1,0);
      }
      state_publisher_->msg_.velocity = odom_vel_;
      state_publisher_->unlockAndPublish();
      last_state_publish_time_ = current_time_;
    }
  }

  void Pr2Odometry::publish()
  {
    if(fabs((last_publish_time_ - current_time_).toSec()) < 
        expected_publish_time_)
      return;

    if(odometry_publisher_->trylock()) {
      getOdometryMessage(odometry_publisher_->msg_);
      odometry_publisher_->unlockAndPublish();
      last_publish_time_ = current_time_;
    }
  }

  void Pr2Odometry::publishTransform()
  {
    if(fabs((last_transform_publish_time_ - current_time_).toSec()) < 
        expected_publish_time_)
      return;
    if(transform_publisher_->trylock()) {
      double x(0.), y(0.0), yaw(0.0), vx(0.0), vy(0.0), vyaw(0.0);
      this->getOdometry(x, y, yaw, vx, vy, vyaw);

      geometry_msgs::TransformStamped &out = 
        transform_publisher_->msg_.transforms[0];

      out.header.stamp = current_time_;
      out.header.frame_id =  tf::resolve(tf_prefix_,base_footprint_frame_);
      out.child_frame_id = tf::resolve(tf_prefix_,odom_frame_);
      out.transform.translation.x = -x * cos(yaw) - y * sin(yaw);
      out.transform.translation.y = +x * sin(yaw) - y * cos(yaw);
      out.transform.translation.z = 0;
      tf::Quaternion quat_trans;
      quat_trans.setRPY(0.0, 0.0, -yaw);

      out.transform.rotation.x = quat_trans.x();
      out.transform.rotation.y = quat_trans.y();
      out.transform.rotation.z = quat_trans.z();
      out.transform.rotation.w = quat_trans.w();

      transform_publisher_->unlockAndPublish();
      last_transform_publish_time_ = current_time_;
    }
  }
} // namespace
