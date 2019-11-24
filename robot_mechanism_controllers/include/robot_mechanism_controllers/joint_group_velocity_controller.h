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

#ifndef JOINT_GROUP_VELOCITY_CONTROLLER_H
#define JOINT_GROUP_VELOCITY_CONTROLLER_H

/**
   @class pr2_controller_interface::JointGroupVelocityController
   @author Stuart Glaser
   @brief Joint Group Velocity Controller

   This controller controls velocity using a pid loop.

   @section ROS ROS interface

   @param type Must be "JointGroupVelocityController"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around velocity.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (std_msgs::Float64) : The joint velocity to achieve

   Publishes:

   - @b state (robot_mechanism_controllers::JointControllerState) :
     Current state of the controller, including pid error and gains.

*/

#include <ros/node_handle.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include "pr2_controller_interface/controller.h"
#include "control_toolbox/pid.h"
#include "control_toolbox/pid_gains_setter.h"

// Services
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

//Realtime publisher
#include <pr2_controllers_msgs/JointControllerState.h>
#include <pr2_controllers_msgs/JointControllerStateArray.h>
#include <realtime_tools/realtime_publisher.h>

namespace controller
{

class JointGroupVelocityController : public pr2_controller_interface::Controller
{
public:

  JointGroupVelocityController();
  ~JointGroupVelocityController();

  bool init(pr2_mechanism_model::RobotState *robot, const std::vector< std::string> &joint_names, const control_toolbox::Pid &pid);
  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param double pos Velocity command to issue
   */
  void setCommand(std::vector<double> cmd);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
  void getCommand(std::vector<double> & cmd);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */

  virtual void starting();
  virtual void update();

  void getGains(control_toolbox::Pid &pid, double &p, double &i, double &d, double &i_max, double &i_min);
  //void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

  std::vector< std::string > getJointName();
  unsigned int n_joints_;

private:
  ros::NodeHandle node_;
  pr2_mechanism_model::RobotState *robot_;                  /**< Pointer to robot structure. */
  std::vector<control_toolbox::Pid> pid_controllers_;           /**< Internal PID controller. */
  std::vector<pr2_mechanism_model::JointState*> joints_;           /**< Joint we're controlling. */
  ros::Time last_time_;                              /**< Last time stamp of update. */
  int loop_count_;

  realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;     /**< Last commanded position. */

  friend class JointGroupVelocityControllerNode;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      pr2_controllers_msgs::JointControllerStateArray> > controller_state_publisher_ ;

  ros::Subscriber sub_command_;
  void setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
};

} // namespace

#endif
