
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
#ifndef PROJECTOR_CONTROLLER_H
#define PROJECTOR_CONTROLLER_H

#include <ros/node_handle.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/robot.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Header.h>
#include <boost/scoped_ptr.hpp>

namespace controller
{
class ProjectorController : public pr2_controller_interface::Controller
{
public:
  ProjectorController();

  ~ProjectorController();

  void update();
  void starting();
  void stopping();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

private:
  pr2_mechanism_model::RobotState * robot_;
  pr2_hardware_interface::Projector *projector_;

  uint32_t old_rising_;
  uint32_t old_falling_;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      std_msgs::Header> > rising_edge_pub_, falling_edge_pub_;

  ros::NodeHandle node_handle_;

  // Configuration of controller.
  std::string actuator_name_;

  double current_setting_;

  double start_time_; /// @todo KILLME FIXME
};

};

#endif

