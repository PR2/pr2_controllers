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

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <actionlib/server/action_server.h>

#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

class ControlHead
{
private:
  typedef actionlib::ActionServer<pr2_controllers_msgs::PointHeadAction> PHAS;
  typedef PHAS::GoalHandle GoalHandle;
public:
  ControlHead(const ros::NodeHandle &n)
    : node_(n),
      action_server_(node_, "point_head_action",
                     boost::bind(&ControlHead::goalCB, this, _1),
                     boost::bind(&ControlHead::cancelCB, this, _1)),
      has_active_goal_(false)
  {
    ros::NodeHandle pn("~");
    pn.param("pan_link", pan_link_, std::string("head_pan_link"));
    pn.param("tilt_link", tilt_link_, std::string("head_tilt_link"));
    pn.param("success_angle_threshold", success_angle_threshold_, 0.1);

    // \todo Need to actually look these joints up
    pan_joint_ = "head_pan_joint";
    tilt_joint_ = "head_tilt_joint";

    std::vector<std::string> frames;
    frames.push_back(pan_link_);
    frames.push_back(tilt_link_);

    // Connects to the controller
    pub_controller_command_ =
      node_.advertise<trajectory_msgs::JointTrajectory>("command", 2);
    sub_controller_state_ =
      node_.subscribe("state", 1, &ControlHead::controllerStateCB, this);
    cli_query_traj_ =
      node_.serviceClient<pr2_controllers_msgs::QueryTrajectoryState>("query_state");

    watchdog_timer_ = node_.createTimer(ros::Duration(1.0), &ControlHead::watchdog, this);
  }


  void goalCB(GoalHandle gh)
  {
    // Before we do anything, we need to know that name of the pan_link's parent.
    if (pan_parent_.empty())
    {
      for (int i = 0; i < 10; ++i)
      {
        try {
          tf_.getParent(pan_link_, ros::Time(), pan_parent_);
          break;
        }
        catch (const tf::TransformException &ex) {}
        ros::Duration(0.5).sleep();
      }

      if (pan_parent_.empty())
      {
        ROS_ERROR("Could not get parent of %s in the TF tree", pan_link_.c_str());
        gh.setRejected();
        return;
      }
    }


    std::vector<double> q_goal(2);  // [pan, tilt]

    // Transforms the target point into the pan and tilt links.
    const geometry_msgs::PointStamped &target = gh.getGoal()->target;
    bool ret1 = false, ret2 = false;
    try {
      ros::Time now = ros::Time::now();
      std::string error_msg;
      ret1 = tf_.waitForTransform(pan_parent_, target.header.frame_id, target.header.stamp,
                                 ros::Duration(5.0), ros::Duration(0.01), &error_msg);
      ret2 = tf_.waitForTransform(pan_link_, target.header.frame_id, target.header.stamp,
                                   ros::Duration(5.0), ros::Duration(0.01), &error_msg);

      // Performs IK to determine the desired joint angles

      // Transforms the target into the pan and tilt frames
      tf::Stamped<tf::Point> target_point, target_in_tilt;
      tf::pointStampedMsgToTF(target, target_point);
      tf_.transformPoint(pan_parent_, target_point, target_in_pan_);
      tf_.transformPoint(pan_link_, target_point, target_in_tilt);

      // Computes the desired joint positions.
      q_goal[0] = atan2(target_in_pan_.y(), target_in_pan_.x());
      q_goal[1] = atan2(-target_in_tilt.z(),
                        sqrt(pow(target_in_tilt.x(),2) + pow(target_in_tilt.y(),2)));
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR("Transform failure (%d,%d): %s", ret1, ret2, ex.what());
      gh.setRejected();
      return;
    }

    // Queries where the movement should start.
    pr2_controllers_msgs::QueryTrajectoryState traj_state;
    traj_state.request.time = ros::Time::now() + ros::Duration(0.01);
    if (!cli_query_traj_.call(traj_state))
    {
      ROS_ERROR("Service call to query controller trajectory failed.");
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
      double dist = sqrt(pow(q_goal[0] - traj_state.response.position[0], 2) +
                         pow(q_goal[1] - traj_state.response.position[1], 2));
      ros::Duration limit_from_velocity(dist / gh.getGoal()->max_velocity);
      if (limit_from_velocity > min_duration)
        min_duration = limit_from_velocity;
    }

    // Computes the command to send to the trajectory controller.
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = traj_state.request.time;

    traj.joint_names.push_back(pan_joint_);
    traj.joint_names.push_back(tilt_joint_);

    traj.points.resize(2);
    traj.points[0].positions = traj_state.response.position;
    traj.points[0].velocities = traj_state.response.velocity;
    traj.points[0].time_from_start = ros::Duration(0.0);
    traj.points[1].positions = q_goal;
    traj.points[1].velocities.push_back(0);
    traj.points[1].velocities.push_back(0);
    traj.points[1].time_from_start = ros::Duration(min_duration);

    pub_controller_command_.publish(traj);
  }


private:
  std::string pan_link_;
  std::string tilt_link_;
  std::string pan_joint_;
  std::string tilt_joint_;

  ros::NodeHandle node_;
  tf::TransformListener tf_;
  ros::Publisher pub_controller_command_;
  ros::Subscriber sub_controller_state_;
  ros::Subscriber command_sub_;
  ros::ServiceClient cli_query_traj_;
  ros::Timer watchdog_timer_;

  PHAS action_server_;
  bool has_active_goal_;
  GoalHandle active_goal_;
  tf::Stamped<tf::Point> target_in_pan_;
  std::string pan_parent_;
  double success_angle_threshold_;

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
        empty.joint_names.push_back(pan_joint_);
        empty.joint_names.push_back(tilt_joint_);
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
      empty.joint_names.push_back(pan_joint_);
      empty.joint_names.push_back(tilt_joint_);
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

    //! \todo Support frames that are not the pan link itself
    try
    {
      tf::Stamped<tf::Vector3> origin(tf::Vector3(0,0,0), msg->header.stamp, tilt_link_);
      tf::Stamped<tf::Vector3> forward(tf::Vector3(1,0,0), msg->header.stamp, tilt_link_);
      tf_.waitForTransform(pan_parent_, tilt_link_, msg->header.stamp, ros::Duration(1.0));
      tf_.transformPoint(pan_parent_, origin, origin);
      tf_.transformPoint(pan_parent_, forward, forward);
      pr2_controllers_msgs::PointHeadFeedback feedback;
      feedback.pointing_angle_error =
        (forward - origin).angle(target_in_pan_ - origin);
      active_goal_.publishFeedback(feedback);

      if (feedback.pointing_angle_error < success_angle_threshold_)
      {
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR("Could not transform: %s", ex.what());
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_head_action");
  ros::NodeHandle node;
  ControlHead ch(node);
  ros::spin();
  return 0;
}
