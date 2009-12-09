/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

// Author: Stuart Glaser

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <robot_mechanism_controllers/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
private:
  typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;
public:
  JointTrajectoryExecuter(ros::NodeHandle &n) :
    node_(n),
    action_server_(ros::NodeHandle()/*node_*/, "action",
                   boost::bind(&JointTrajectoryExecuter::goalCB, this, _1),
                   boost::bind(&JointTrajectoryExecuter::cancelCB, this, _1)),
    has_active_goal_(false)
  {
    using namespace XmlRpc;

    // Gets all of the joints
    XmlRpc::XmlRpcValue joint_names;
    if (!node_.getParam("joints", joint_names))
    {
      ROS_FATAL("No joints given. (namespace: %s)", node_.getNamespace().c_str());
      exit(1);
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_FATAL("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
      exit(1);
    }
    for (int i = 0; i < joint_names.size(); ++i)
    {
      XmlRpcValue &name_value = joint_names[i];
      if (name_value.getType() != XmlRpcValue::TypeString)
      {
        ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)",
                  node_.getNamespace().c_str());
        exit(1);
      }

      joint_names_.push_back((std::string)name_value);
    }


    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      std::string ns = std::string("constraints/") + joint_names_[i];
      double g, t;
      node_.param(ns + "/goal", g, -1.0);
      node_.param(ns + "/trajectory", t, -1.0);
      goal_constraints_[joint_names_[i]] = g;
      trajectory_constraints_[joint_names_[i]] = t;
    }

   pub_controller_command_ =
      node_.advertise<trajectory_msgs::JointTrajectory>("controller_command", 1);
    sub_controller_state_ =
      node_.subscribe("controller_state", 1, &JointTrajectoryExecuter::controllerStateCB, this);

    watchdog_timer_ = node_.createTimer(ros::Duration(1.0), &JointTrajectoryExecuter::watchdog, this);
  }

  ~JointTrajectoryExecuter()
  {
    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
    watchdog_timer_.stop();
  }

private:

  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
  {
    if (a.size() != b.size())
      return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
      if (count(b.begin(), b.end(), a[i]) != 1)
        return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
      if (count(a.begin(), a.end(), b[i]) != 1)
        return false;
    }

    return true;
  }

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
        empty.joint_names = joint_names_;
        pub_controller_command_.publish(empty);

        // Marks the current goal as aborted.
        active_goal_.setAborted();
        has_active_goal_ = false;
      }
    }
  }

  void goalCB(GoalHandle gh)
  {
    // Ensures that the joints in the goal match the joints we are commanding.
    if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names))
    {
      ROS_ERROR("Joints on incoming goal don't match our joints");
      gh.setRejected();
      return;
    }

    // Cancels the currently active goal.
    if (has_active_goal_)
    {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;

    // Sends the trajectory along to the controller
    current_traj_ = active_goal_.getGoal()->trajectory;
    pub_controller_command_.publish(current_traj_);
  }

  void cancelCB(GoalHandle gh)
  {
    if (active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }


  ros::NodeHandle node_;
  JTAS action_server_;
  ros::Publisher pub_controller_command_;
  ros::Subscriber sub_controller_state_;
  ros::Timer watchdog_timer_;

  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::JointTrajectory current_traj_;


  std::vector<std::string> joint_names_;
  std::map<std::string,double> goal_constraints_;
  std::map<std::string,double> trajectory_constraints_;

  std::map<std::string,double> goal_thresholds_;

  robot_mechanism_controllers::JointTrajectoryControllerStateConstPtr last_controller_state_;
  void controllerStateCB(const robot_mechanism_controllers::JointTrajectoryControllerStateConstPtr &msg)
  {
    last_controller_state_ = msg;
    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
      return;
    if (current_traj_.points.empty())
      return;
    if (now < current_traj_.header.stamp + current_traj_.points[0].time_from_start)
      return;

    if (!setsEqual(joint_names_, msg->joint_names))
    {
      ROS_ERROR("Joint names from the controller don't match our joint names.");
      return;
    }

    int last = current_traj_.points.size() - 1;
    ros::Time end_time = current_traj_.header.stamp + current_traj_.points[last].time_from_start;

    // Verifies that the controller has stayed within the trajectory constraints.

    if (now < end_time)
    {
      // Checks that the controller is inside the trajectory constraints.
      for (size_t i = 0; i < msg->joint_names.size(); ++i)
      {
        double abs_error = fabs(msg->error.positions[i]);
        double constraint = trajectory_constraints_[msg->joint_names[i]];
        if (constraint >= 0 && abs_error > constraint)
        {
          // Stops the controller.
          trajectory_msgs::JointTrajectory empty;
          empty.joint_names = joint_names_;
          pub_controller_command_.publish(empty);

          active_goal_.setAborted();
          has_active_goal_ = false;
          ROS_WARN("Aborting because we would up outside the trajectory constraints");
          return;
        }
      }
    }
    else
    {
      // Checks that we have ended inside the goal constraints
      bool inside_goal_constraints = true;
      for (size_t i = 0; i < msg->joint_names.size(); ++i)
      {
        double abs_error = fabs(msg->error.positions[i]);
        double goal_constraint = goal_constraints_[msg->joint_names[i]];
        if (goal_constraint >= 0 && abs_error > goal_constraint)
          inside_goal_constraints = false;
      }

      if (inside_goal_constraints)
        active_goal_.setSucceeded();
      else
      {
        ROS_WARN("Aborting because we wound up outside the goal constraints");
        active_goal_.setAborted();
      }

      has_active_goal_ = false;
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_trajectory_action");
  ros::NodeHandle node("~");
  JointTrajectoryExecuter jte(node);

  ros::spin();

  return 0;
}
