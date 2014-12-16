/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include "pr2_mechanism_controllers/pr2_base_controller2.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( controller::Pr2BaseController2, pr2_controller_interface::Controller)

namespace controller {

  const static double EPS = 1e-5;

Pr2BaseController2::Pr2BaseController2()
{
  //init variables
  cmd_vel_.linear.x = 0;
  cmd_vel_.linear.y = 0;
  cmd_vel_.angular.z = 0;

  desired_vel_.linear.x = 0;
  desired_vel_.linear.y = 0;
  desired_vel_.angular.z = 0;

  cmd_vel_t_.linear.x = 0;
  cmd_vel_t_.linear.y = 0;
  cmd_vel_t_.angular.z = 0;

  new_cmd_available_ = false;
  last_publish_time_ = ros::Time(0.0);

  pthread_mutex_init(&pr2_base_controller_lock_, NULL);
}

Pr2BaseController2::~Pr2BaseController2()
{
  cmd_sub_.shutdown();
  cmd_sub_deprecated_.shutdown();
}

bool Pr2BaseController2::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  if(!base_kinematics_.init(robot,n))
    return false;
  node_ = n;
  state_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_mechanism_controllers::BaseControllerState2>(n, base_kinematics_.name_ + "/state", 1));

  int num_joints = base_kinematics_.num_wheels_ + base_kinematics_.num_casters_;
  state_publisher_->msg_.joint_names.resize(num_joints);
  state_publisher_->msg_.joint_velocity_commanded.resize(num_joints);
  state_publisher_->msg_.joint_velocity_measured.resize(num_joints);
  state_publisher_->msg_.joint_effort_measured.resize(num_joints);
  state_publisher_->msg_.joint_command.resize(num_joints);
  state_publisher_->msg_.joint_effort_commanded.resize(num_joints);
  state_publisher_->msg_.joint_error.resize(num_joints);
  state_publisher_->msg_.joint_effort_error.resize(num_joints);

  //Get params from param server
  node_.param<double> ("max_translational_velocity", max_translational_velocity_,0.5);
  node_.param<double> ("max_rotational_velocity", max_rotational_velocity_, 10.0); //0.5
  node_.param<double> ("max_translational_acceleration/x", max_accel_.linear.x, .2);
  node_.param<double> ("max_translational_acceleration/y", max_accel_.linear.y, .2);
  node_.param<double> ("max_rotational_acceleration", max_accel_.angular.z, 10.0); //0.2

  node_.param<double> ("kp_caster_steer", kp_caster_steer_, 80.0);
  node_.param<double> ("timeout", timeout_, 1.0);
  node_.param<double> ("state_publish_rate", state_publish_rate_,2.0);
  if(state_publish_rate_ <= 0.0)
  {
    publish_state_ = false;
    state_publish_time_ = 0.0;
  }
  else
  {
    publish_state_ = true;
    state_publish_time_ = 1.0/state_publish_rate_;
  }

  //  cmd_sub_deprecated_ = root_handle_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Pr2BaseController2::commandCallback, this);
  cmd_sub_ = node_.subscribe<geometry_msgs::Twist>("command", 1, &Pr2BaseController2::commandCallback, this);

  //casters
  caster_controller_.resize(base_kinematics_.num_casters_);
  caster_position_pid_.resize(base_kinematics_.num_casters_);
  for(int i = 0; i < base_kinematics_.num_casters_; i++)
  {
    control_toolbox::Pid p_i_d;
    state_publisher_->msg_.joint_names[i] = base_kinematics_.caster_[i].joint_name_;
    if(!p_i_d.init(ros::NodeHandle(node_, base_kinematics_.caster_[i].joint_name_+"/velocity_controller")))
    {
      ROS_ERROR("Could not initialize pid for %s",base_kinematics_.caster_[i].joint_name_.c_str());
      return false;
    }

    if(!caster_position_pid_[i].init(ros::NodeHandle(node_, base_kinematics_.caster_[i].joint_name_+"/position_controller")))
    {
      ROS_ERROR("Could not initialize position pid controller for %s",base_kinematics_.caster_[i].joint_name_.c_str());
      return false;
    }
    caster_controller_[i].reset(new JointVelocityController());
    if(!caster_controller_[i]->init(base_kinematics_.robot_state_, base_kinematics_.caster_[i].joint_name_, p_i_d))
    {
      ROS_ERROR("Could not initialize pid for %s",base_kinematics_.caster_[i].joint_name_.c_str());
      return false;
    }
    if (!caster_controller_[i]->joint_state_->calibrated_)
    {
      ROS_ERROR("Caster joint \"%s\" not calibrated (namespace: %s)",
                base_kinematics_.caster_[i].joint_name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
  }
  //wheels
  wheel_pid_controllers_.resize(base_kinematics_.num_wheels_);
  //  wheel_controller_.resize(base_kinematics_.num_wheels_);
  for(int j = 0; j < base_kinematics_.num_wheels_; j++)
  {
    control_toolbox::Pid p_i_d;
    state_publisher_->msg_.joint_names[j + base_kinematics_.num_casters_] = base_kinematics_.wheel_[j].joint_name_;
    if(!wheel_pid_controllers_[j].init(ros::NodeHandle(node_,base_kinematics_.wheel_[j].joint_name_)))
    {
      ROS_ERROR("Could not initialize pid for %s",base_kinematics_.wheel_[j].joint_name_.c_str());
      return false;
    }
    /*    wheel_controller_[j].reset(new JointVelocityController());
   if(!wheel_controller_[j]->init(base_kinematics_.robot_state_, base_kinematics_.wheel_[j].joint_name_, p_i_d))
   {
      ROS_ERROR("Could not initialize joint velocity controller for %s",base_kinematics_.wheel_[j].joint_name_.c_str());
      return false;
      }*/
  }
  for(int i = 0; i < base_kinematics_.num_casters_; ++i)
  {
    if(!base_kinematics_.caster_[i].joint_->calibrated_)
    {
      ROS_ERROR("The Base controller could not start because the casters were not calibrated. Relaunch the base controller after you see the caster calibration finish.");
      return false; // Casters are not calibrated
    }
  }

  if (!((filters::MultiChannelFilterBase<double>&)caster_vel_filter_).configure(base_kinematics_.num_casters_, std::string("caster_velocity_filter"), node_)){
     ROS_ERROR("BaseController: could not configure velocity filters for casters");
     return false;
  }
  if (!((filters::MultiChannelFilterBase<double>&)wheel_vel_filter_).configure(base_kinematics_.num_wheels_, std::string("wheel_velocity_filter"), node_)){
     ROS_ERROR("BaseController: could not configure velocity filters for wheels");
     return false;
  }
  filtered_velocity_.resize(base_kinematics_.num_casters_);
  filtered_wheel_velocity_.resize(base_kinematics_.num_wheels_);
  return true;
}

// Set the base velocity command
void Pr2BaseController2::setCommand(const geometry_msgs::Twist &cmd_vel)
{
  double vel_mag = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
  double clamped_vel_mag = filters::clamp(vel_mag,-max_translational_velocity_, max_translational_velocity_);
  if(vel_mag > EPS)
  {
    cmd_vel_t_.linear.x = cmd_vel.linear.x * clamped_vel_mag / vel_mag;
    cmd_vel_t_.linear.y = cmd_vel.linear.y * clamped_vel_mag / vel_mag;
  }
  else
  {
    cmd_vel_t_.linear.x = 0.0;
    cmd_vel_t_.linear.y = 0.0;
  }
  cmd_vel_t_.angular.z = filters::clamp(cmd_vel.angular.z, -max_rotational_velocity_, max_rotational_velocity_);
  cmd_received_timestamp_ = base_kinematics_.robot_state_->getTime();

  ROS_DEBUG("BaseController:: command received: %f %f %f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
  ROS_DEBUG("BaseController:: command current: %f %f %f", cmd_vel_.linear.x,cmd_vel_.linear.y,cmd_vel_.angular.z);
  ROS_DEBUG("BaseController:: clamped vel: %f", clamped_vel_mag);
  ROS_DEBUG("BaseController:: vel: %f", vel_mag);

  for(int i=0; i < (int) base_kinematics_.num_wheels_; i++)
  {
    ROS_DEBUG("BaseController:: wheel speed cmd:: %d %f",i,(base_kinematics_.wheel_[i].direction_multiplier_*base_kinematics_.wheel_[i].wheel_speed_cmd_));
  }
  for(int i=0; i < (int) base_kinematics_.num_casters_; i++)
  {
    ROS_DEBUG("BaseController:: caster speed cmd:: %d %f",i,(base_kinematics_.caster_[i].steer_velocity_desired_));
  }
  new_cmd_available_ = true;
}

geometry_msgs::Twist Pr2BaseController2::interpolateCommand(const geometry_msgs::Twist &start, const geometry_msgs::Twist &end, const geometry_msgs::Twist &max_rate, const double &dT)
{
  geometry_msgs::Twist result;
  geometry_msgs::Twist alpha;
  double delta(0), max_delta(0);

  delta = end.linear.x - start.linear.x;
  max_delta = max_rate.linear.x * dT;
  if(fabs(delta) <= max_delta || max_delta < EPS)
    alpha.linear.x = 1;
  else
    alpha.linear.x = max_delta / fabs(delta);

  delta = end.linear.y - start.linear.y;
  max_delta = max_rate.linear.y * dT;
  if(fabs(delta) <= max_delta || max_delta < EPS)
    alpha.linear.y = 1;
  else
    alpha.linear.y = max_delta / fabs(delta);

  delta = end.angular.z - start.angular.z;
  max_delta = max_rate.angular.z * dT;
  if(fabs(delta) <= max_delta || max_delta < EPS)
    alpha.angular.z = 1;
  else
    alpha.angular.z = max_delta / fabs(delta);

  double alpha_min = alpha.linear.x;
  if(alpha.linear.y < alpha_min)
    alpha_min = alpha.linear.y;
  if(alpha.angular.z < alpha_min)
    alpha_min = alpha.angular.z;

  result.linear.x = start.linear.x + alpha_min * (end.linear.x - start.linear.x);
  result.linear.y = start.linear.y + alpha_min * (end.linear.y - start.linear.y);
  result.angular.z = start.angular.z + alpha_min * (end.angular.z - start.angular.z);
  return result;
}

geometry_msgs::Twist Pr2BaseController2::getCommand()// Return the current velocity command
{
  geometry_msgs::Twist cmd_vel;
  pthread_mutex_lock(&pr2_base_controller_lock_);
  cmd_vel.linear.x = cmd_vel_.linear.x;
  cmd_vel.linear.y = cmd_vel_.linear.y;
  cmd_vel.angular.z = cmd_vel_.angular.z;
  pthread_mutex_unlock(&pr2_base_controller_lock_);
  return cmd_vel;
}

void Pr2BaseController2::starting()
{
  last_time_ = base_kinematics_.robot_state_->getTime();
  cmd_received_timestamp_ = base_kinematics_.robot_state_->getTime();
  for(int i = 0; i < base_kinematics_.num_casters_; i++)
  {
    caster_controller_[i]->starting();
  }
  for(int j = 0; j < base_kinematics_.num_wheels_; j++)
  {
    //    wheel_controller_[j]->starting();
  }
}

void Pr2BaseController2::update()
{
  ros::Time current_time = base_kinematics_.robot_state_->getTime();
  double dT = std::min<double>((current_time - last_time_).toSec(), base_kinematics_.MAX_DT_);

  if(new_cmd_available_)
  {
    if(pthread_mutex_trylock(&pr2_base_controller_lock_) == 0)
    {
      desired_vel_.linear.x = cmd_vel_t_.linear.x;
      desired_vel_.linear.y = cmd_vel_t_.linear.y;
      desired_vel_.angular.z = cmd_vel_t_.angular.z;
      new_cmd_available_ = false;
      pthread_mutex_unlock(&pr2_base_controller_lock_);
    }
  }

  if((current_time - cmd_received_timestamp_).toSec() > timeout_)
  {
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.angular.z = 0;
  }
  else
    cmd_vel_ = interpolateCommand(cmd_vel_, desired_vel_, max_accel_, dT);

  computeJointCommands(dT);

  setJointCommands();

  updateJointControllers();

  if(publish_state_)
    publishState(current_time);

  last_time_ = current_time;

}

void Pr2BaseController2::publishState(const ros::Time &time)
{
  if((time - last_publish_time_).toSec()  < state_publish_time_)
  {
    return;
  }

  if(state_publisher_->trylock())
  {
    state_publisher_->msg_.command.linear.x  = cmd_vel_.linear.x;
    state_publisher_->msg_.command.linear.y  = cmd_vel_.linear.y;
    state_publisher_->msg_.command.angular.z = cmd_vel_.angular.z;

    for(int i = 0; i < base_kinematics_.num_casters_; i++)
    {
      state_publisher_->msg_.joint_names[i] = base_kinematics_.caster_[i].joint_name_;
      state_publisher_->msg_.joint_velocity_measured[i] = base_kinematics_.caster_[i].joint_->velocity_;
      state_publisher_->msg_.joint_command[i]= base_kinematics_.caster_[i].steer_angle_desired_;
      state_publisher_->msg_.joint_error[i]  = base_kinematics_.caster_[i].joint_->position_ - base_kinematics_.caster_[i].steer_angle_desired_;

      state_publisher_->msg_.joint_effort_measured[i]  = base_kinematics_.caster_[i].joint_->measured_effort_;
      state_publisher_->msg_.joint_effort_commanded[i] = base_kinematics_.caster_[i].joint_->commanded_effort_;
      state_publisher_->msg_.joint_effort_error[i]     = base_kinematics_.caster_[i].joint_->measured_effort_ - base_kinematics_.caster_[i].joint_->commanded_effort_;
    }
    for(int i = 0; i < base_kinematics_.num_wheels_; i++)
    {
      state_publisher_->msg_.joint_names[i+base_kinematics_.num_casters_] = base_kinematics_.wheel_[i].joint_name_;
      state_publisher_->msg_.joint_velocity_commanded[i+base_kinematics_.num_casters_] = base_kinematics_.wheel_[i].wheel_speed_cmd_;
      state_publisher_->msg_.joint_velocity_measured[i+base_kinematics_.num_casters_] = base_kinematics_.wheel_[i].joint_->velocity_;
      state_publisher_->msg_.joint_command[i+base_kinematics_.num_casters_]= base_kinematics_.wheel_[i].joint_->velocity_-base_kinematics_.wheel_[i].wheel_speed_cmd_;
      state_publisher_->msg_.joint_error[i+base_kinematics_.num_casters_]    = base_kinematics_.wheel_[i].wheel_speed_cmd_;

      state_publisher_->msg_.joint_effort_measured[i+base_kinematics_.num_casters_]  = base_kinematics_.wheel_[i].joint_->measured_effort_;
      state_publisher_->msg_.joint_effort_commanded[i+base_kinematics_.num_casters_] = base_kinematics_.wheel_[i].joint_->commanded_effort_;
      state_publisher_->msg_.joint_effort_error[i+base_kinematics_.num_casters_]     = base_kinematics_.wheel_[i].joint_->measured_effort_ - base_kinematics_.wheel_[i].joint_->commanded_effort_;
    }
    state_publisher_->unlockAndPublish();
    last_publish_time_ = time;
  }
}

void Pr2BaseController2::computeJointCommands(const double &dT)
{
  base_kinematics_.computeWheelPositions();

  computeDesiredCasterSteer(dT);

  computeDesiredWheelSpeeds(dT);
}

void Pr2BaseController2::setJointCommands()
{
  setDesiredCasterSteer();

  setDesiredWheelSpeeds();
}

void Pr2BaseController2::computeDesiredCasterSteer(const double &dT)
{
  geometry_msgs::Twist result;

  double steer_angle_desired(0.0), steer_angle_desired_m_pi(0.0);
  double error_steer(0.0), error_steer_m_pi(0.0);
  double trans_vel = sqrt(cmd_vel_.linear.x * cmd_vel_.linear.x + cmd_vel_.linear.y * cmd_vel_.linear.y);

  for(int i = 0; i < base_kinematics_.num_casters_; i++)
  {
    filtered_velocity_[i] = 0.0 - base_kinematics_.caster_[i].joint_->velocity_;
  }
  caster_vel_filter_.update(filtered_velocity_,filtered_velocity_);

  for(int i = 0; i < base_kinematics_.num_casters_; i++)
  {
    result = base_kinematics_.pointVel2D(base_kinematics_.caster_[i].offset_, cmd_vel_);
    if(trans_vel < EPS && fabs(cmd_vel_.angular.z) < EPS)
    {
      steer_angle_desired = base_kinematics_.caster_[i].steer_angle_stored_;
    }
    else
    {
      steer_angle_desired = atan2(result.linear.y, result.linear.x);
      base_kinematics_.caster_[i].steer_angle_stored_ = steer_angle_desired;
    }
    steer_angle_desired_m_pi = angles::normalize_angle(steer_angle_desired + M_PI);
    error_steer = angles::shortest_angular_distance(
          base_kinematics_.caster_[i].joint_->position_,
          steer_angle_desired);
    error_steer_m_pi = angles::shortest_angular_distance(
          base_kinematics_.caster_[i].joint_->position_,
          steer_angle_desired_m_pi);

    if(fabs(error_steer_m_pi) < fabs(error_steer))
    {
      error_steer = error_steer_m_pi;
      steer_angle_desired = steer_angle_desired_m_pi;
    }
    base_kinematics_.caster_[i].steer_angle_desired_ = steer_angle_desired;
    double command = caster_position_pid_[i].computeCommand(error_steer,
          filtered_velocity_[i], ros::Duration(dT));
    base_kinematics_.caster_[i].joint_->commanded_effort_ = command;

    base_kinematics_.caster_[i].caster_position_error_ = error_steer;
  }
}

void Pr2BaseController2::setDesiredCasterSteer()
{
  for(int i = 0; i < base_kinematics_.num_casters_; i++)
  {
    //    caster_controller_[i]->setCommand(base_kinematics_.caster_[i].steer_velocity_desired_);
  }
}

void Pr2BaseController2::computeDesiredWheelSpeeds(const double &dT)
{
  geometry_msgs::Twist wheel_point_velocity;
  geometry_msgs::Twist wheel_point_velocity_projected;
  geometry_msgs::Twist wheel_caster_steer_component;
  geometry_msgs::Twist caster_2d_velocity;

  caster_2d_velocity.linear.x = 0;
  caster_2d_velocity.linear.y = 0;
  caster_2d_velocity.angular.z = 0;

  for(int i = 0; i < base_kinematics_.num_wheels_; i++)
  {
    filtered_wheel_velocity_[i] = base_kinematics_.wheel_[i].joint_->velocity_;
  }
  wheel_vel_filter_.update(filtered_wheel_velocity_,filtered_wheel_velocity_);

  double steer_angle_actual = 0;
  for(int i = 0; i < (int) base_kinematics_.num_wheels_; i++)
  {
    base_kinematics_.wheel_[i].updatePosition();
    //    caster_2d_velocity.angular.z = base_kinematics_.wheel_[i].parent_->steer_velocity_desired_;
    caster_2d_velocity.angular.z = 0.0 - base_kinematics_.wheel_[i].parent_->caster_position_error_;
    steer_angle_actual = base_kinematics_.wheel_[i].parent_->joint_->position_;
    wheel_point_velocity = base_kinematics_.pointVel2D(base_kinematics_.wheel_[i].position_, cmd_vel_);
    wheel_caster_steer_component = base_kinematics_.pointVel2D(base_kinematics_.wheel_[i].offset_, caster_2d_velocity);

    double costh = cos(-steer_angle_actual);
    double sinth = sin(-steer_angle_actual);

    wheel_point_velocity_projected.linear.x = costh * wheel_point_velocity.linear.x - sinth * wheel_point_velocity.linear.y;
    wheel_point_velocity_projected.linear.y = sinth * wheel_point_velocity.linear.x + costh * wheel_point_velocity.linear.y;
    base_kinematics_.wheel_[i].wheel_speed_cmd_ = (wheel_point_velocity_projected.linear.x) / (base_kinematics_.wheel_[i].wheel_radius_);
    double command = wheel_pid_controllers_[i].computeCommand(
          - wheel_caster_steer_component.linear.x/base_kinematics_.wheel_[i].wheel_radius_,
          base_kinematics_.wheel_[i].wheel_speed_cmd_ - filtered_wheel_velocity_[i],
          ros::Duration(dT));
    base_kinematics_.wheel_[i].joint_->commanded_effort_ = command;
  }
}

void Pr2BaseController2::setDesiredWheelSpeeds()
{
  /*  for(int i = 0; i < (int) base_kinematics_.num_wheels_; i++)
  {
   wheel_controller_[i]->setCommand(base_kinematics_.wheel_[i].direction_multiplier_ * base_kinematics_.wheel_[i].wheel_speed_cmd_);
    }*/
}

void Pr2BaseController2::updateJointControllers()
{
  /*  for(int i = 0; i < base_kinematics_.num_wheels_; i++)
      wheel_controller_[i]->update();
  for(int i = 0; i < base_kinematics_.num_casters_; i++)
  caster_controller_[i]->update();*/
}

void Pr2BaseController2::commandCallback(const geometry_msgs::TwistConstPtr& msg)
{
  pthread_mutex_lock(&pr2_base_controller_lock_);
  base_vel_msg_ = *msg;
  this->setCommand(base_vel_msg_);
  pthread_mutex_unlock(&pr2_base_controller_lock_);
}
} // namespace
