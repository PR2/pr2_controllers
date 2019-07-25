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

#include <cmath>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

class SingleJointPositionNode
{
private:
  typedef actionlib::ActionServer<pr2_controllers_msgs::SingleJointPositionAction> SJPAS;
  typedef SJPAS::GoalHandle GoalHandle;
public:
  SingleJointPositionNode(const ros::NodeHandle &n)
    : node_(n),
      action_server_(node_, "position_joint_action",
                     boost::bind(&SingleJointPositionNode::goalCB, this, _1),
                     boost::bind(&SingleJointPositionNode::cancelCB, this, _1),
                     false),
      has_active_goal_(false)
  {
    ros::NodeHandle pn("~");
    if (!pn.getParam("joint", joint_))
    {
      ROS_FATAL("No joint given.");
      exit(1);
    }

    pn.param("goal_threshold", goal_threshold_, 0.1);
    pn.param("max_acceleration", max_accel_, -1.0);

    // Connects to the controller
    pub_controller_command_ =
      node_.advertise<trajectory_msgs::JointTrajectory>("command", 2);
    sub_controller_state_ =
      node_.subscribe("state", 1, &SingleJointPositionNode::controllerStateCB, this);
    cli_query_traj_ =
      node_.serviceClient<pr2_controllers_msgs::QueryTrajectoryState>("query_state");

    if (!cli_query_traj_.waitForExistence(ros::Duration(10.0)))
    {
      ROS_WARN("The controller does not appear to be ready (the query_state service is not available)");
    }

    // Starts the action server
    action_server_.start();

    watchdog_timer_ = node_.createTimer(ros::Duration(1.0), &SingleJointPositionNode::watchdog, this);
  }


  //void pointHeadCB(const geometry_msgs::PointStampedConstPtr &msg)
  void goalCB(GoalHandle gh)
  {
    // Queries where the movement should start.
    pr2_controllers_msgs::QueryTrajectoryState traj_state;
    traj_state.request.time = ros::Time::now() + ros::Duration(0.01);
    if (!cli_query_traj_.call(traj_state))
    {
      ROS_ERROR("Service call to query controller trajectory failed.  Service: (%s)",
                cli_query_traj_.getService().c_str());
      gh.setRejected();
      return;
    }

    if (has_active_goal_)
    {
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;

    // Computes the duration of the movement.

    ros::Duration min_duration(0.01);

    if (gh.getGoal()->min_duration > min_duration)
        min_duration = gh.getGoal()->min_duration;

    // Determines if we need to increase the duration of the movement
    // in order to enforce a maximum velocity.
    if (gh.getGoal()->max_velocity > 0)
    {
      // Very approximate velocity limiting.
      double dist = fabs(gh.getGoal()->position - traj_state.response.position[0]);
      ros::Duration limit_from_velocity(dist / gh.getGoal()->max_velocity);
      //! \todo Use max_accel_ to compute the velocity correctly.
      if (limit_from_velocity > min_duration)
        min_duration = limit_from_velocity;
    }

    // Computes the command to send to the trajectory controller.
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = traj_state.request.time;

    traj.joint_names.push_back(joint_);

    traj.points.resize(2);
    traj.points[0].positions = traj_state.response.position;
    traj.points[0].velocities = traj_state.response.velocity;
    traj.points[0].time_from_start = ros::Duration(0.0);
    traj.points[1].positions.push_back(gh.getGoal()->position);
    traj.points[1].velocities.push_back(0);
    traj.points[1].time_from_start = ros::Duration(min_duration);

    pub_controller_command_.publish(traj);
  }


private:
  std::string joint_;
  double max_accel_;
  double goal_threshold_;

  ros::NodeHandle node_;
  ros::Publisher pub_controller_command_;
  ros::Subscriber sub_controller_state_;
  ros::Subscriber command_sub_;
  ros::ServiceClient cli_query_traj_;
  ros::Timer watchdog_timer_;

  SJPAS action_server_;
  bool has_active_goal_;
  GoalHandle active_goal_;

  void watchdog(const ros::TimerEvent &e)
  {
    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
                 (now - last_controller_state_->header.stamp).toSec());
      }

      if (should_abort)
      {
        // Stops the controller.
        trajectory_msgs::JointTrajectory empty;
        empty.joint_names.push_back(joint_);
        pub_controller_command_.publish(empty);

        // Marks the current goal as aborted.
        active_goal_.setAborted();
        has_active_goal_ = false;
      }
    }
  }

  void cancelCB(GoalHandle gh)
  {
    if (active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names.push_back(joint_);
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }

  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr last_controller_state_;
  void controllerStateCB(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)
  {
    last_controller_state_ = msg;
    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
      return;

    pr2_controllers_msgs::SingleJointPositionFeedback feedback;
    feedback.header.stamp = msg->header.stamp;
    feedback.position = msg->actual.positions[0];
    feedback.velocity = msg->actual.velocities[0];
    feedback.error = msg->error.positions[0];
    active_goal_.publishFeedback(feedback);

    if (fabs(msg->actual.positions[0] - active_goal_.getGoal()->position) < goal_threshold_)
    {
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_joint_action_node");
  ros::NodeHandle node;
  SingleJointPositionNode a(node);
  ros::spin();
  return 0;
}
