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
 */

#ifndef CASTER_CALIBRATION_CONTROLLER_H
#define CASTER_CALIBRATION_CONTROLLER_H

#include "ros/node_handle.h"
#include "pr2_mechanism_controllers/caster_controller.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/Empty.h"

namespace controller {

class CasterCalibrationController : public pr2_controller_interface::Controller
{
public:
  CasterCalibrationController();
  ~CasterCalibrationController();

  //virtual bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);
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

  ros::NodeHandle node_;
  pr2_mechanism_model::RobotState *robot_;

  enum { INITIALIZED, BEGINNING, MOVING, CALIBRATED };
  int state_;

  double search_velocity_;
  bool original_switch_state_;

  pr2_hardware_interface::Actuator *actuator_;
  pr2_mechanism_model::JointState *joint_, *wheel_l_joint_, *wheel_r_joint_;
  pr2_mechanism_model::Transmission *transmission_;

  controller::CasterController cc_;

  ros::Time last_publish_time_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Empty> > pub_calibrated_;
};

} // namespace

#endif
