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
  Sample config:
  <controller type="WristCalibrationController" name="cal_wrist">
    <calibrate transmission="wrist_trans"
               actuator_l="wrist_l_motor" actuator_r="wrist_r_motor"
               flex_joint="wrist_flex_joint" roll_joint="wrist_roll_joint"
               velocity="0.6" />
    <pid p="3.0" i="0.2" d="0" iClamp="2.0" />
  </controller>

 * Author: Stuart Glaser
 */

#ifndef WRIST_CALIBRATION_CONTROLLER_H
#define WRIST_CALIBRATION_CONTROLLER_H

#include "robot_mechanism_controllers/joint_velocity_controller.h"
#include "realtime_tools/realtime_publisher.h"
#include "pr2_mechanism_model/wrist_transmission.h"
#include "std_msgs/Empty.h"

namespace controller {

class WristCalibrationController : public pr2_controller_interface::Controller
{
public:
  WristCalibrationController();
  ~WristCalibrationController();

  virtual bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update();

  bool calibrated() { return state_ == CALIBRATED; }
  void beginCalibration()
  {
    if (state_ == INITIALIZED)
      state_ = BEGINNING;
  }

protected:

  enum { INITIALIZED, BEGINNING, MOVING_FLEX, MOVING_ROLL, CALIBRATED };
  int state_;

  pr2_mechanism_model::RobotState *robot_;
  ros::NodeHandle node_;
  ros::Time last_publish_time_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Empty> > pub_calibrated_;

  double roll_search_velocity_;
  double flex_search_velocity_;
  bool original_switch_state_;

  // Tracks the actuator positions for when the optical switch occurred.
  double flex_switch_l_, flex_switch_r_;
  double roll_switch_l_, roll_switch_r_;

  double prev_actuator_l_position_, prev_actuator_r_position_;

  pr2_hardware_interface::Actuator *actuator_l_, *actuator_r_;
  pr2_mechanism_model::JointState *flex_joint_, *roll_joint_;
  pr2_mechanism_model::Transmission *transmission_;

  controller::JointVelocityController vc_flex_, vc_roll_;
};


}

#endif
