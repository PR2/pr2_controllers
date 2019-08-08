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

/*!
  \author Stuart Glaser

  \class pr2_controller_interface::JointTrajectoryActionController

*/

#ifndef JOINT_TRAJECTORY_ACTION_CONTROLLER_H__
#define JOINT_TRAJECTORY_ACTION_CONTROLLER_H__

#include <vector>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/node_handle.h>

#include <actionlib/server/action_server.h>
#include <control_toolbox/limited_proxy.h>
#include <control_toolbox/pid.h>
#include <filters/filter_chain.h>
#include <pr2_controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>


#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>


namespace controller {

template <class Action>
class RTServerGoalHandle
{
private:
  ACTION_DEFINITION(Action);

  //typedef actionlib::ActionServer<Action>::GoalHandle GoalHandle;
  typedef actionlib::ServerGoalHandle<Action> GoalHandle;
#if ((actionlib_VERSION_MAJOR == 1) && (actionlib_VERSION_MINOR < 12)) || (actionlib_VERSION_MAJOR < 1)
  typedef boost::shared_ptr<Result> ResultPtr;
  typedef boost::shared_ptr<Feedback> FeedbackPtr;
#endif

  uint8_t state_;

  bool req_abort_;
  bool req_succeed_;
  ResultConstPtr req_result_;

public:
  GoalHandle gh_;
  ResultPtr preallocated_result_;  // Preallocated so it can be used in realtime
  FeedbackPtr preallocated_feedback_;

 RTServerGoalHandle(GoalHandle &gh, const ResultPtr &preallocated_result = ResultPtr((Result*)NULL))
   : req_abort_(false), req_succeed_(false), gh_(gh), preallocated_result_(preallocated_result)
  {
    if (!preallocated_result_)
      preallocated_result_.reset(new Result);
    if (!preallocated_feedback_)
      preallocated_feedback_.reset(new Feedback);
  }

  void setAborted(ResultConstPtr result = ResultConstPtr((Result*)NULL))
  {
    if (!req_succeed_ && !req_abort_)
    {
      req_result_ = result;
      req_abort_ = true;
    }
  }

  void setSucceeded(ResultConstPtr result = ResultConstPtr((Result*)NULL))
  {
    if (!req_succeed_ && !req_abort_)
    {
      req_result_ = result;
      req_succeed_ = true;
    }
  }

  bool valid()
  {
    return gh_.getGoal() != NULL;
  }

  void runNonRT(const ros::TimerEvent &te)
  {
    using namespace actionlib_msgs;
    if (valid())
    {
      actionlib_msgs::GoalStatus gs = gh_.getGoalStatus();
      if (req_abort_ && gs.status == GoalStatus::ACTIVE)
      {
        if (req_result_)
          gh_.setAborted(*req_result_);
        else
          gh_.setAborted();
      }
      else if (req_succeed_ && gs.status == GoalStatus::ACTIVE)
      {
        if (req_result_)
          gh_.setSucceeded(*req_result_);
        else
          gh_.setSucceeded();
      }
    }
  }
};

class JointTolerance
{
public:
  JointTolerance(double _position = 0, double _velocity = 0, double _acceleration = 0) :
    position(_position), velocity(_velocity), acceleration(_acceleration)
  {
  }

  bool violated(double p_err, double v_err = 0, double a_err = 0) const
  {
    return
      (position > 0 && fabs(p_err) > position) ||
      (velocity > 0 && fabs(v_err) > velocity) ||
      (acceleration > 0 && fabs(a_err) > acceleration);
  }

  double position;
  double velocity;
  double acceleration;
};


class JointTrajectoryActionController : public pr2_controller_interface::Controller
{
  // Action typedefs for the original PR2 specific joint trajectory action
  typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;
  typedef RTServerGoalHandle<pr2_controllers_msgs::JointTrajectoryAction> RTGoalHandle;

  // Action typedefs for the new follow joint trajectory action
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
  typedef FJTAS::GoalHandle GoalHandleFollow;
  typedef RTServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RTGoalHandleFollow;
    
public:

  JointTrajectoryActionController();
  ~JointTrajectoryActionController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  void starting();
  void update();

private:
  int loop_count_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time last_time_;
  std::vector<pr2_mechanism_model::JointState*> joints_;
  std::vector<double> masses_;  // Rough estimate of joint mass, used for feedforward control
  std::vector<control_toolbox::Pid> pids_;
  std::vector<bool> proxies_enabled_;
  std::vector<control_toolbox::LimitedProxy> proxies_;

  std::vector<JointTolerance> default_trajectory_tolerance_;
  std::vector<JointTolerance> default_goal_tolerance_;
  double default_goal_time_constraint_;

  /*
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;
  std::vector<double> goal_constraints_;
  std::vector<double> trajectory_constraints_;
  */
  std::vector<boost::shared_ptr<filters::FilterChain<double> > > output_filters_;

  ros::NodeHandle node_;

  void commandCB(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  ros::Subscriber sub_command_;

  bool queryStateService(pr2_controllers_msgs::QueryTrajectoryState::Request &req,
                         pr2_controllers_msgs::QueryTrajectoryState::Response &resp);
  ros::ServiceServer serve_query_state_;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      pr2_controllers_msgs::JointTrajectoryControllerState> > controller_state_publisher_;

  boost::scoped_ptr<JTAS> action_server_;
  boost::scoped_ptr<FJTAS> action_server_follow_;
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void goalCBFollow(GoalHandleFollow gh);
  void cancelCBFollow(GoalHandleFollow gh);
  ros::Timer goal_handle_timer_;

  boost::shared_ptr<RTGoalHandle> rt_active_goal_;
  boost::shared_ptr<RTGoalHandleFollow> rt_active_goal_follow_;

  // ------ Mechanisms for passing the trajectory into realtime

  void commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &traj,
                         boost::shared_ptr<RTGoalHandle> gh = boost::shared_ptr<RTGoalHandle>((RTGoalHandle*)NULL),
                         boost::shared_ptr<RTGoalHandleFollow> gh_follow = boost::shared_ptr<RTGoalHandleFollow>((RTGoalHandleFollow*)NULL));

  void preemptActiveGoal();

  // coef[0] + coef[1]*t + ... + coef[5]*t^5
  struct Spline
  {
    std::vector<double> coef;

    Spline() : coef(6, 0.0) {}
  };

  struct Segment
  {
    double start_time;
    double duration;
    std::vector<Spline> splines;
    
    std::vector<JointTolerance> trajectory_tolerance;
    std::vector<JointTolerance> goal_tolerance;
    double goal_time_tolerance;

    boost::shared_ptr<RTGoalHandle> gh;
    boost::shared_ptr<RTGoalHandleFollow> gh_follow;  // Goal handle for the newer FollowJointTrajectory action
  };
  typedef std::vector<Segment> SpecifiedTrajectory;

  realtime_tools::RealtimeBox<
    boost::shared_ptr<const SpecifiedTrajectory> > current_trajectory_box_;

  // Holds the trajectory that we are currently following.  The mutex
  // guarding current_trajectory_ is locked from within realtime, so
  // it may only be locked for a bounded duration.
  //boost::shared_ptr<const SpecifiedTrajectory> current_trajectory_;
  //boost::recursive_mutex current_trajectory_lock_RT_;

  std::vector<double> q, qd, qdd;  // Preallocated in init

  // Samples, but handling time bounds.  When the time is past the end
  // of the spline duration, the position is the last valid position,
  // and the derivatives are all 0.
  static void sampleSplineWithTimeBounds(const std::vector<double>& coefficients, double duration, double time,
                                         double& position, double& velocity, double& acceleration);
};

}

#endif
