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

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include "ros/node_handle.h"
#include "pr2_mechanism_controllers/caster_controller.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/Empty.h"
#include "pr2_controllers_msgs/QueryCalibrationState.h"

namespace controller {

class CasterCalibrationController : public pr2_controller_interface::Controller
{
public:
  CasterCalibrationController();
  ~CasterCalibrationController();

  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update();

  bool isCalibrated(pr2_controllers_msgs::QueryCalibrationState::Request& req, pr2_controllers_msgs::QueryCalibrationState::Response& resp);

protected:

  ros::NodeHandle node_;
  pr2_mechanism_model::RobotState *robot_;

  enum { INITIALIZED, BEGINNING, MOVING, CALIBRATED };
  int state_;

  double search_velocity_;
  bool original_switch_state_;
  double original_position_;

  ros::Time beginning_;
  int unstick_iter_;

  pr2_hardware_interface::Actuator *actuator_;
  pr2_mechanism_model::JointState *joint_, *wheel_l_joint_, *wheel_r_joint_;
  boost::shared_ptr<pr2_mechanism_model::Transmission> transmission_;

  // Preallocated, for use in update()
  std::vector<pr2_hardware_interface::Actuator*> fake_as;
  std::vector<pr2_mechanism_model::JointState*> fake_js;

  controller::CasterController cc_;

  ros::Time last_publish_time_;
  ros::ServiceServer is_calibrated_srv_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Empty> > pub_calibrated_;
};

} // namespace

#endif
