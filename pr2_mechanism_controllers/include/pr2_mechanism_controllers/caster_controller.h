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
 * Author: Stuart Glaser

 Example config:

 <controller type="CasterController" name="a_caster">
   <joints caster="caster_joint" wheel_l="wheel_left_joint" wheel_r="wheel_right_joint">
   <caster_pid p="5.0" i="0.0" d="0.0" iClamp="0.0" />
   <wheel_pid p="4.0" i="0.0" d="0.0" iClamp="0.0" />
 </controller>

 YAML config
\verbatim
caster_fl:
  type: CasterController
  caster_pid: {p: 6.0}
  wheel_pid: {p: 4.0}
  joints:
    caster: fl_caster_rotation_joint
    wheel_l: fl_caster_l_wheel_joint
    wheel_r: fl_caster_r_wheel_joint
\endverbatim

 */

#ifndef CASTER_CONTROLLER_H
#define CASTER_CONTROLLER_H

#include "ros/node_handle.h"

#include "pr2_controller_interface/controller.h"
#include "pr2_mechanism_model/robot.h"
#include "control_toolbox/pid.h"
#include "robot_mechanism_controllers/joint_velocity_controller.h"
#include "std_msgs/Float64.h"
#include <boost/thread/condition.hpp>

namespace controller {

class CasterController : public pr2_controller_interface::Controller
{
public:
  const static double WHEEL_RADIUS;
  const static double WHEEL_OFFSET;

  CasterController();
  ~CasterController();

  bool init(pr2_mechanism_model::RobotState *robot_state,
            const std::string &caster_joint,
            const std::string &wheel_l_joint, const std::string &wheel_r_joint,
            const control_toolbox::Pid &caster_pid, const control_toolbox::Pid &wheel_pid);
  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  void update();

  double steer_velocity_;
  double drive_velocity_;

  double getSteerPosition() { return caster_->position_; }
  double getSteerVelocity() { return caster_->velocity_; }

  pr2_mechanism_model::JointState *caster_;

private:
  ros::NodeHandle node_;
  JointVelocityController caster_vel_, wheel_l_vel_, wheel_r_vel_;
  ros::Subscriber steer_cmd_;
  ros::Subscriber drive_cmd_;

  void setSteerCB(const std_msgs::Float64ConstPtr& msg);
  void setDriveCB(const std_msgs::Float64ConstPtr& msg);
};

}

#endif
