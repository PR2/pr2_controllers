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

#include "pr2_mechanism_controllers/pr2_base_controller.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(Pr2BaseController, controller::Pr2BaseController, pr2_controller_interface::Controller)

namespace controller {

Pr2BaseController::Pr2BaseController()
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

//  state_publisher_ = NULL;

  last_publish_time_ = ros::Time(0.0);

  pthread_mutex_init(&pr2_base_controller_lock_, NULL);
}

Pr2BaseController::~Pr2BaseController()
{
}

bool Pr2BaseController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  if(!base_kin_.init(robot,n))
    return false;
  node_ = n;
//  if(state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
//    delete state_publisher_;
  state_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_mechanism_controllers::BaseControllerState>(base_kin_.name_ + "/state", 1));
  state_publisher_->msg_.set_joint_name_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_stall_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_speed_error_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_speed_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_speed_filtered_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_commanded_effort_size(base_kin_.num_wheels_ + base_kin_.num_casters_);
  state_publisher_->msg_.set_joint_measured_effort_size(base_kin_.num_wheels_ + base_kin_.num_casters_);

//    joy_sub_ = ros_node_.subscribe(joy_listen_topic, 1, &AntiCollisionBaseController::joyCallBack, this);

  //Get params from param server
  node_.param<double> ("max_vel/vx", max_vel_.linear.x, .5);
  node_.param<double> ("max_vel/vy", max_vel_.linear.y, .5);
  node_.param<double> ("max_vel/omegaz", max_vel_.angular.z, 10.0); //0.5
  node_.param<double> ("max_accel/ax", max_accel_.linear.x, .2);
  node_.param<double> ("max_accel/ay", max_accel_.linear.y, .2);
  node_.param<double> ("max_accel/alphaz", max_accel_.linear.z, 10.0); //0.2
  node_.param<double> ("caster_speed_threshold", caster_speed_threshold_, 0.2);
  node_.param<double> ("caster_position_error_threshold", caster_position_error_threshold_, 0.05);
  node_.param<double> ("wheel_speed_threshold", wheel_speed_threshold_, 0.2);
  node_.param<double> ("caster_effort_threshold", caster_effort_threshold_, 3.45);
  node_.param<double> ("wheel_effort_threshold", wheel_effort_threshold_, 3.45);
  node_.param<double> ("kp_wheel_steer_", kp_wheel_steer_, 2.0);
  node_.param<double> ("alpha_stall", alpha_stall_, 0.5);
  node_.param<double> ("kp_caster_steer_", kp_caster_steer_, 40.0);
  node_.param<double> ("timeout", timeout_, 1.0);
  node_.param<double> ("eps", eps_, 1e-5);
  node_.param<double> ("cmd_vel_trans_eps", cmd_vel_trans_eps_, 1e-5);
  node_.param<double> ("cmd_vel_rot_eps", cmd_vel_rot_eps_, 1e-5);
  node_.param<double> ("state_publish_time", state_publish_time_, 0.5);

  node_.param<std::string> ("cmd_topic", cmd_topic_, "/cmd_vel");
  cmd_sub_ = node_.subscribe<geometry_msgs::Twist>(cmd_topic_, 1, &Pr2BaseController::CmdBaseVelReceived, this);

  //casters
  caster_controller_.resize(base_kin_.num_casters_);
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    control_toolbox::Pid p_i_d;
    state_publisher_->msg_.joint_name[i] = base_kin_.caster_[i].joint_name_;
    if(!p_i_d.init(ros::NodeHandle(node_, base_kin_.caster_[i].joint_name_)))
    {
      ROS_ERROR("Could not initialize pid for %s",base_kin_.caster_[i].joint_name_.c_str());
      return false;
    }
    caster_controller_[i].reset(new JointVelocityController());
    if(!caster_controller_[i]->init(base_kin_.robot_state_, base_kin_.caster_[i].joint_name_, p_i_d))
    {
      ROS_ERROR("Could not initialize pid for %s",base_kin_.caster_[i].joint_name_.c_str());
      return false;
    }
  }
  //wheels
  wheel_controller_.resize(base_kin_.num_wheels_);
  for(int j = 0; j < base_kin_.num_wheels_; j++)
  {
    control_toolbox::Pid p_i_d;
    state_publisher_->msg_.joint_name[j + base_kin_.num_casters_] = base_kin_.wheel_[j].joint_name_;
    if(!p_i_d.init(ros::NodeHandle(node_,base_kin_.wheel_[j].joint_name_)))
    {
      ROS_ERROR("Could not initialize pid for %s",base_kin_.wheel_[j].joint_name_.c_str());
      return false;
    }
    wheel_controller_[j].reset(new JointVelocityController());
   if(!wheel_controller_[j]->init(base_kin_.robot_state_, base_kin_.wheel_[j].joint_name_, p_i_d))
   {
      ROS_ERROR("Could not initialize joint velocity controller for %s",base_kin_.wheel_[j].joint_name_.c_str());
      return false;
   }
  }
  return true;
}

bool Pr2BaseController::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
//  base_kin_.initXml(robot, config);
  ros::NodeHandle n(config->Attribute("name"));
  return init(robot, n);
}

// Set the base velocity command
void Pr2BaseController::setCommand(geometry_msgs::Twist cmd_vel)
{
  double vel_mag = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
  double clamped_vel_mag = filters::clamp(vel_mag, -(max_vel_.linear.x + max_vel_.linear.y) / 2.0, (max_vel_.linear.x + max_vel_.linear.y) / 2.0);
  if(vel_mag > eps_)
  {
    cmd_vel_t_.linear.x = cmd_vel.linear.x * clamped_vel_mag / vel_mag;
    cmd_vel_t_.linear.y = cmd_vel.linear.y * clamped_vel_mag / vel_mag;
  }
  else
  {
    cmd_vel_t_.linear.x = 0.0;
    cmd_vel_t_.linear.y = 0.0;
  }
  cmd_vel_t_.angular.z = filters::clamp(cmd_vel.angular.z, -max_vel_.angular.z, max_vel_.angular.z);
  cmd_received_timestamp_ = base_kin_.robot_state_->getTime();

  ROS_DEBUG("BaseController:: command received: %f %f %f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
  ROS_DEBUG("BaseController:: command current: %f %f %f", cmd_vel_.linear.x,cmd_vel_.linear.y,cmd_vel_.angular.z);
  ROS_DEBUG("BaseController:: clamped vel: %f", clamped_vel_mag);
  ROS_DEBUG("BaseController:: vel: %f", vel_mag);

  for(int i=0; i < (int) base_kin_.num_wheels_; i++)
  {
    ROS_DEBUG("BaseController:: wheel speed cmd:: %d %f",i,(base_kin_.wheel_[i].direction_multiplier_*base_kin_.wheel_[i].wheel_speed_cmd_));
  }
  for(int i=0; i < (int) base_kin_.num_casters_; i++)
  {
    ROS_DEBUG("BaseController:: caster speed cmd:: %d %f",i,(base_kin_.caster_[i].steer_velocity_desired_));
  }
  new_cmd_available_ = true;
}

geometry_msgs::Twist Pr2BaseController::interpolateCommand(geometry_msgs::Twist start, geometry_msgs::Twist end, geometry_msgs::Twist max_rate, double dT)
{
  geometry_msgs::Twist result;
  geometry_msgs::Twist alpha;
  double delta(0), max_delta(0);

  delta = end.linear.x - start.linear.x;
  max_delta = max_rate.linear.x * dT;
  if(fabs(delta) <= max_delta || max_delta < eps_)
    alpha.linear.x = 1;
  else
    alpha.linear.x = max_delta / fabs(delta);

  delta = end.linear.y - start.linear.y;
  max_delta = max_rate.linear.y * dT;
  if(fabs(delta) <= max_delta || max_delta < eps_)
    alpha.linear.y = 1;
  else
    alpha.linear.y = max_delta / fabs(delta);

  delta = end.angular.z - start.angular.z;
  max_delta = max_rate.angular.z * dT;
  if(fabs(delta) <= max_delta || max_delta < eps_)
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

geometry_msgs::Twist Pr2BaseController::getCommand()// Return the current velocity command
{
  geometry_msgs::Twist cmd_vel;
  pthread_mutex_lock(&pr2_base_controller_lock_);
  cmd_vel.linear.x = cmd_vel_.linear.x;
  cmd_vel.linear.y = cmd_vel_.linear.y;
  cmd_vel.angular.z = cmd_vel_.angular.z;
  pthread_mutex_unlock(&pr2_base_controller_lock_);
  return cmd_vel;
}

bool Pr2BaseController::starting()
{
  for(int i = 0; i < base_kin_.num_casters_; ++i)
  {
    if(!base_kin_.caster_[i].joint_->calibrated_)
    {
      ROS_ERROR("The Base controller could not start because the casters were not calibrated. Relaunch the base controller after you see the caster calibration finish.");
      return false; // Casters are not calibrated
    }
  }

  last_time_ = base_kin_.robot_state_->getTime();
  cmd_received_timestamp_ = base_kin_.robot_state_->getTime();
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    caster_controller_[i]->starting();
  }
  for(int j = 0; j < base_kin_.num_wheels_; j++)
  {
    wheel_controller_[j]->starting();
  }
  return true;
}

void Pr2BaseController::update()
{

  ros::Time current_time = base_kin_.robot_state_->getTime();
  double dT = std::min<double>((current_time - last_time_).toSec(), base_kin_.MAX_DT_);

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

  computeJointCommands();

  setJointCommands();

  updateJointControllers();

  computeStall();

  publishState(current_time);

  last_time_ = current_time;

}

void Pr2BaseController::publishState(ros::Time time)
{

  if((time - last_publish_time_).toSec()  < state_publish_time_)
  {
    return;
  }
  if(state_publisher_->trylock())
  {
    state_publisher_->msg_.command_vx = cmd_vel_.linear.x;
    state_publisher_->msg_.command_vy = cmd_vel_.linear.y;
    state_publisher_->msg_.command_vw = cmd_vel_.angular.z;
    for(int i = 0; i < base_kin_.num_casters_; i++)
    {
      state_publisher_->msg_.joint_speed[i] = base_kin_.caster_[i].caster_speed_;
      state_publisher_->msg_.joint_speed_filtered[i] = base_kin_.caster_[i].caster_speed_filtered_;
      state_publisher_->msg_.joint_speed_error[i] = base_kin_.caster_[i].caster_speed_error_;
      state_publisher_->msg_.joint_stall[i] = base_kin_.caster_[i].caster_stuck_;
      state_publisher_->msg_.joint_commanded_effort[i] = base_kin_.caster_[i].joint_->commanded_effort_;
      state_publisher_->msg_.joint_measured_effort[i] = base_kin_.caster_[i].joint_->measured_effort_;
    }
    for(int i = 0; i < base_kin_.num_wheels_; i++)
    {
      state_publisher_->msg_.joint_speed[i + base_kin_.num_casters_] = base_kin_.wheel_[i].wheel_speed_actual_;
      state_publisher_->msg_.joint_speed_filtered[i + base_kin_.num_casters_] = base_kin_.wheel_[i].wheel_speed_filtered_;
      state_publisher_->msg_.joint_speed_error[i + base_kin_.num_casters_] = base_kin_.wheel_[i].wheel_speed_error_;
      state_publisher_->msg_.joint_stall[i + base_kin_.num_casters_] = base_kin_.wheel_[i].wheel_stuck_;
      state_publisher_->msg_.joint_commanded_effort[i + base_kin_.num_casters_] = base_kin_.wheel_[i].joint_->commanded_effort_;
      state_publisher_->msg_.joint_measured_effort[i + base_kin_.num_casters_] = base_kin_.wheel_[i].joint_->measured_effort_;
    }

    state_publisher_->unlockAndPublish();
    last_publish_time_ = time;
  }
}

void Pr2BaseController::computeJointCommands()
{
  base_kin_.computeWheelPositions();

  computeDesiredCasterSteer();

  computeDesiredWheelSpeeds();
}

void Pr2BaseController::setJointCommands()
{
  setDesiredCasterSteer();

  setDesiredWheelSpeeds();
}

void Pr2BaseController::computeDesiredCasterSteer()
{
  geometry_msgs::Twist result;

  double steer_angle_desired(0.0), steer_angle_desired_m_pi(0.0);
  double error_steer(0.0), error_steer_m_pi(0.0);
  double trans_vel = sqrt(cmd_vel_.linear.x * cmd_vel_.linear.x + cmd_vel_.linear.y * cmd_vel_.linear.y);
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    result = base_kin_.pointVel2D(base_kin_.caster_[i].offset_, cmd_vel_);
    if(trans_vel < cmd_vel_trans_eps_ && fabs(cmd_vel_.angular.z) < cmd_vel_rot_eps_)
    {
      steer_angle_desired = base_kin_.caster_[i].steer_angle_stored_;
    }
    else
    {
      steer_angle_desired = atan2(result.linear.y, result.linear.x);
      base_kin_.caster_[i].steer_angle_stored_ = steer_angle_desired;
    }
    steer_angle_desired_m_pi = angles::normalize_angle(steer_angle_desired + M_PI);
    error_steer = angles::shortest_angular_distance(steer_angle_desired, base_kin_.caster_[i].joint_->position_);
    error_steer_m_pi = angles::shortest_angular_distance(steer_angle_desired_m_pi, base_kin_.caster_[i].joint_->position_);

    if(fabs(error_steer_m_pi) < fabs(error_steer))
    {
      error_steer = error_steer_m_pi;
      steer_angle_desired = steer_angle_desired_m_pi;
    }
    base_kin_.caster_[i].steer_velocity_desired_ = -kp_caster_steer_ * error_steer;
    base_kin_.caster_[i].caster_position_error_ = error_steer;
  }
}

void Pr2BaseController::setDesiredCasterSteer()
{
  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    caster_controller_[i]->setCommand(base_kin_.caster_[i].steer_velocity_desired_);
  }
}

void Pr2BaseController::computeDesiredWheelSpeeds()
{
  geometry_msgs::Twist wheel_point_velocity;
  geometry_msgs::Twist wheel_point_velocity_projected;
  geometry_msgs::Twist wheel_caster_steer_component;
  geometry_msgs::Twist caster_2d_velocity;

  caster_2d_velocity.linear.x = 0;
  caster_2d_velocity.linear.y = 0;
  caster_2d_velocity.angular.z = 0;

  double steer_angle_actual = 0;
  for(int i = 0; i < (int) base_kin_.num_wheels_; i++)
  {
    base_kin_.wheel_[i].updatePosition();
    caster_2d_velocity.angular.z = kp_wheel_steer_ * base_kin_.wheel_[i].parent_->steer_velocity_desired_;
    steer_angle_actual = base_kin_.wheel_[i].parent_->joint_->position_;
    wheel_point_velocity = base_kin_.pointVel2D(base_kin_.wheel_[i].position_, cmd_vel_);
    wheel_caster_steer_component = base_kin_.pointVel2D(base_kin_.wheel_[i].offset_, caster_2d_velocity);

    double costh = cos(-steer_angle_actual);
    double sinth = sin(-steer_angle_actual);

    wheel_point_velocity_projected.linear.x = costh * wheel_point_velocity.linear.x - sinth * wheel_point_velocity.linear.y;
    wheel_point_velocity_projected.linear.y = sinth * wheel_point_velocity.linear.x + costh * wheel_point_velocity.linear.y;
    base_kin_.wheel_[i].wheel_speed_cmd_ = (wheel_point_velocity_projected.linear.x + wheel_caster_steer_component.linear.x) / (base_kin_.wheel_radius_ * base_kin_.wheel_[i].wheel_radius_scaler_);
  }
}

void Pr2BaseController::setDesiredWheelSpeeds()
{
  for(int i = 0; i < (int) base_kin_.num_wheels_; i++)
  {
    wheel_controller_[i]->setCommand(base_kin_.wheel_[i].direction_multiplier_ * base_kin_.wheel_[i].wheel_speed_cmd_);
  }
}

void Pr2BaseController::updateJointControllers()
{
  for(int i = 0; i < base_kin_.num_wheels_; i++)
    wheel_controller_[i]->update();
  for(int i = 0; i < base_kin_.num_casters_; i++)
    caster_controller_[i]->update();
}

void Pr2BaseController::computeStall()
{

  for(int i = 0; i < base_kin_.num_casters_; i++)
  {
    base_kin_.caster_[i].caster_speed_error_ = fabs(base_kin_.caster_[i].joint_->velocity_ - base_kin_.caster_[i].steer_velocity_desired_);
    base_kin_.caster_[i].caster_speed_filtered_ = alpha_stall_ * base_kin_.caster_[i].caster_speed_filtered_ + (1 - alpha_stall_) * base_kin_.caster_[i].joint_->velocity_;//low pass filter
    base_kin_.caster_[i].caster_speed_ = base_kin_.caster_[i].joint_->velocity_;

    if(fabs(base_kin_.caster_[i].caster_speed_) < caster_speed_threshold_ && fabs(base_kin_.caster_[i].caster_position_error_) > caster_position_error_threshold_ && fabs(base_kin_.caster_[i].joint_->measured_effort_) > caster_effort_threshold_)
    {
      base_kin_.caster_[i].caster_stuck_ = 1;
    }
    else
    {
      base_kin_.caster_[i].caster_stuck_ = 0;
    }
  }

  for(int j = 0; j < base_kin_.num_wheels_; j++)
  {
    base_kin_.wheel_[j].wheel_speed_error_ = fabs(base_kin_.wheel_[j].joint_->velocity_ - base_kin_.wheel_[j].wheel_speed_cmd_);
    base_kin_.wheel_[j].wheel_speed_filtered_ = alpha_stall_ * base_kin_.wheel_[j].wheel_speed_filtered_ + (1 - alpha_stall_) * base_kin_.wheel_[j].wheel_speed_actual_;
    if(fabs(base_kin_.wheel_[j].wheel_speed_filtered_) < wheel_speed_threshold_ && fabs(base_kin_.wheel_[j].joint_->measured_effort_) > wheel_effort_threshold_)
    {
      base_kin_.wheel_[j].wheel_stuck_ = 1;
    }
    else
    {
      base_kin_.wheel_[j].wheel_stuck_ = 0;
    }
  }
}

void Pr2BaseController::CmdBaseVelReceived(const geometry_msgs::TwistConstPtr& msg)
{
  pthread_mutex_lock(&pr2_base_controller_lock_);
  base_vel_msg_ = *msg;
  this->setCommand(base_vel_msg_);
  pthread_mutex_unlock(&pr2_base_controller_lock_);
}
} // namespace
