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

#include <boost/shared_ptr.hpp>

#include "robot_mechanism_controllers/joint_spline_trajectory_controller.h"
#include <sstream>
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( controller::JointSplineTrajectoryController, pr2_controller_interface::Controller)

namespace controller {

// These functions are pulled from the spline_smoother package.
// They've been moved here to avoid depending on packages that aren't
// mature yet.


static inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; i++)
  {
    powers[i] = powers[i-1]*x;
  }
}

static void getQuinticSplineCoefficients(double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc, double time, std::vector<double>& coefficients)
{
  coefficients.resize(6);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.5*end_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  }
  else
  {
    double T[6];
    generatePowers(5, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5*start_acc;
    coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[2] + end_acc*T[2] -
                       12.0*start_vel*T[1] - 8.0*end_vel*T[1]) / (2.0*T[3]);
    coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[2] - 2.0*end_acc*T[2] +
                       16.0*start_vel*T[1] + 14.0*end_vel*T[1]) / (2.0*T[4]);
    coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[2] -
                       6.0*start_vel*T[1] - 6.0*end_vel*T[1]) / (2.0*T[5]);
  }
}

/**
 * \brief Samples a quintic spline segment at a particular time
 */
static void sampleQuinticSpline(const std::vector<double>& coefficients, double time,
    double& position, double& velocity, double& acceleration)
{
  // create powers of time:
  double t[6];
  generatePowers(5, time, t);

  position = t[0]*coefficients[0] +
      t[1]*coefficients[1] +
      t[2]*coefficients[2] +
      t[3]*coefficients[3] +
      t[4]*coefficients[4] +
      t[5]*coefficients[5];

  velocity = t[0]*coefficients[1] +
      2.0*t[1]*coefficients[2] +
      3.0*t[2]*coefficients[3] +
      4.0*t[3]*coefficients[4] +
      5.0*t[4]*coefficients[5];

  acceleration = 2.0*t[0]*coefficients[2] +
      6.0*t[1]*coefficients[3] +
      12.0*t[2]*coefficients[4] +
      20.0*t[3]*coefficients[5];
}

static void getCubicSplineCoefficients(double start_pos, double start_vel,
    double end_pos, double end_vel, double time, std::vector<double>& coefficients)
{
  coefficients.resize(4);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
  }
  else
  {
    double T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
    coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
  }
}


JointSplineTrajectoryController::JointSplineTrajectoryController()
  : loop_count_(0), robot_(NULL)
{
}

JointSplineTrajectoryController::~JointSplineTrajectoryController()
{
  sub_command_.shutdown();
  serve_query_state_.shutdown();
}

bool JointSplineTrajectoryController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  using namespace XmlRpc;
  node_ = n;
  robot_ = robot;

  // Gets all of the joints
  XmlRpc::XmlRpcValue joint_names;
  if (!node_.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                node_.getNamespace().c_str());
      return false;
    }

    pr2_mechanism_model::JointState *j = robot->getJointState((std::string)name_value);
    if (!j) {
      ROS_ERROR("Joint not found: %s. (namespace: %s)",
                ((std::string)name_value).c_str(), node_.getNamespace().c_str());
      return false;
    }
    joints_.push_back(j);
  }

  // Ensures that all the joints are calibrated.
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i]->calibrated_)
    {
      ROS_ERROR("Joint %s was not calibrated (namespace: %s)",
                joints_[i]->joint_->name.c_str(), node_.getNamespace().c_str());
      return false;
    }
  }

  // Sets up pid controllers for all of the joints
  std::string gains_ns;
  if (!node_.getParam("gains", gains_ns))
    gains_ns = node_.getNamespace() + "/gains";
  pids_.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joints_[i]->joint_->name)))
      return false;

  // Creates a dummy trajectory
  boost::shared_ptr<SpecifiedTrajectory> traj_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &traj = *traj_ptr;
  traj[0].start_time = robot_->getTime().toSec();
  traj[0].duration = 0.0;
  traj[0].splines.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    traj[0].splines[j].coef[0] = 0.0;
  current_trajectory_box_.set(traj_ptr);

  sub_command_ = node_.subscribe("command", 1, &JointSplineTrajectoryController::commandCB, this);
  serve_query_state_ = node_.advertiseService(
    "query_state", &JointSplineTrajectoryController::queryStateService, this);

  q.resize(joints_.size());
  qd.resize(joints_.size());
  qdd.resize(joints_.size());

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointTrajectoryControllerState>
    (node_, "state", 1));
  controller_state_publisher_->lock();
  for (size_t j = 0; j < joints_.size(); ++j)
    controller_state_publisher_->msg_.joint_names.push_back(joints_[j]->joint_->name);
  controller_state_publisher_->msg_.desired.positions.resize(joints_.size());
  controller_state_publisher_->msg_.desired.velocities.resize(joints_.size());
  controller_state_publisher_->msg_.desired.accelerations.resize(joints_.size());
  controller_state_publisher_->msg_.actual.positions.resize(joints_.size());
  controller_state_publisher_->msg_.actual.velocities.resize(joints_.size());
  controller_state_publisher_->msg_.error.positions.resize(joints_.size());
  controller_state_publisher_->msg_.error.velocities.resize(joints_.size());
  controller_state_publisher_->unlock();


  return true;
}

void JointSplineTrajectoryController::starting()
{
  last_time_ = robot_->getTime();

  for (size_t i = 0; i < pids_.size(); ++i)
    pids_[i].reset();

  // Creates a "hold current position" trajectory.
  boost::shared_ptr<SpecifiedTrajectory> hold_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &hold = *hold_ptr;
  hold[0].start_time = last_time_.toSec() - 0.001;
  hold[0].duration = 0.0;
  hold[0].splines.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    hold[0].splines[j].coef[0] = joints_[j]->position_;

  current_trajectory_box_.set(hold_ptr);
}

void JointSplineTrajectoryController::update()
{
  // Checks if all the joints are calibrated.

  ros::Time time = robot_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  boost::shared_ptr<const SpecifiedTrajectory> traj_ptr;
  current_trajectory_box_.get(traj_ptr);
  if (!traj_ptr)
    ROS_FATAL("The current trajectory can never be null");

  // Only because this is what the code originally looked like.
  const SpecifiedTrajectory &traj = *traj_ptr;

  // Determines which segment of the trajectory to use.  (Not particularly realtime friendly).
  int seg = -1;
  while (seg + 1 < (int)traj.size() &&
         traj[seg+1].start_time < time.toSec())
  {
    ++seg;
  }

  if (seg == -1)
  {
    if (traj.size() == 0)
      ROS_ERROR("No segments in the trajectory");
    else
      ROS_ERROR("No earlier segments.  First segment starts at %.3lf (now = %.3lf)", traj[0].start_time, time.toSec());
    return;
  }

  // ------ Trajectory Sampling

  for (size_t i = 0; i < q.size(); ++i)
  {
    sampleSplineWithTimeBounds(traj[seg].splines[i].coef, traj[seg].duration,
                               time.toSec() - traj[seg].start_time,
                               q[i], qd[i], qdd[i]);
  }

  // ------ Trajectory Following

  std::vector<double> error(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    error[i] = q[i] - joints_[i]->position_;
    joints_[i]->commanded_effort_ += pids_[i].computeCommand(error[i],
          joints_[i]->velocity_ - qd[i], dt);
  }

  // ------ State publishing

  if (loop_count_ % 10 == 0)
  {
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        controller_state_publisher_->msg_.desired.positions[j] = q[j];
        controller_state_publisher_->msg_.desired.velocities[j] = qd[j];
        controller_state_publisher_->msg_.desired.accelerations[j] = qdd[j];
        controller_state_publisher_->msg_.actual.positions[j] = joints_[j]->position_;
        controller_state_publisher_->msg_.actual.velocities[j] = joints_[j]->velocity_;
        controller_state_publisher_->msg_.error.positions[j] = error[j];
        controller_state_publisher_->msg_.error.velocities[j] = joints_[j]->velocity_ - qd[j];
      }
      controller_state_publisher_->unlockAndPublish();
    }
  }

  ++loop_count_;
}

void JointSplineTrajectoryController::commandCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ros::Time time = last_time_;
  ROS_DEBUG("Figuring out new trajectory at %.3lf, with data from %.3lf",
            time.toSec(), msg->header.stamp.toSec());

  boost::shared_ptr<SpecifiedTrajectory> new_traj_ptr(new SpecifiedTrajectory);
  SpecifiedTrajectory &new_traj = *new_traj_ptr;

  // ------ If requested, performs a stop

  if (msg->points.empty())
  {
    starting();
    return;
  }

  // ------ Correlates the joints we're commanding to the joints in the message

  std::vector<int> lookup(joints_.size(), -1);  // Maps from an index in joints_ to an index in the msg
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    for (size_t k = 0; k < msg->joint_names.size(); ++k)
    {
      if (msg->joint_names[k] == joints_[j]->joint_->name)
      {
        lookup[j] = k;
        break;
      }
    }

    if (lookup[j] == -1)
    {
      ROS_ERROR("Unable to locate joint %s in the commanded trajectory.", joints_[j]->joint_->name.c_str());
      return;
    }
  }

  // ------ Grabs the trajectory that we're currently following.

  boost::shared_ptr<const SpecifiedTrajectory> prev_traj_ptr;
  current_trajectory_box_.get(prev_traj_ptr);
  if (!prev_traj_ptr)
  {
    ROS_FATAL("The current trajectory can never be null");
    return;
  }
  const SpecifiedTrajectory &prev_traj = *prev_traj_ptr;

  // ------ Copies over the segments from the previous trajectory that are still useful.

  // Useful segments are still relevant after the current time.
  int first_useful = -1;
  while (first_useful + 1 < (int)prev_traj.size() &&
         prev_traj[first_useful + 1].start_time <= time.toSec())
  {
    ++first_useful;
  }

  // Useful segments are not going to be completely overwritten by the message's splines.
  int last_useful = -1;
  double msg_start_time;
  if (msg->points.size() > 0)
    msg_start_time = (msg->header.stamp + msg->points[0].time_from_start).toSec();
  else
    msg_start_time = std::max(time.toSec(), msg->header.stamp.toSec());

  while (last_useful + 1 < (int)prev_traj.size() &&
         prev_traj[last_useful + 1].start_time < msg_start_time)
  {
    ++last_useful;
  }

  if (last_useful < first_useful)
    first_useful = last_useful;

  // Copies over the old segments that were determined to be useful.
  for (int i = std::max(first_useful,0); i <= last_useful; ++i)
  {
    new_traj.push_back(prev_traj[i]);
  }

  // We always save the last segment so that we know where to stop if
  // there are no new segments.
  if (new_traj.size() == 0)
    new_traj.push_back(prev_traj[prev_traj.size() - 1]);

  // ------ Determines when and where the new segments start

  // Finds the end conditions of the final segment
  Segment &last = new_traj[new_traj.size() - 1];
  std::vector<double> prev_positions(joints_.size());
  std::vector<double> prev_velocities(joints_.size());
  std::vector<double> prev_accelerations(joints_.size());

  ROS_DEBUG("Initial conditions for new set of splines:");
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    sampleSplineWithTimeBounds(last.splines[i].coef, last.duration,
                               msg->header.stamp.toSec() - last.start_time,
                               prev_positions[i], prev_velocities[i], prev_accelerations[i]);
    ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)", prev_positions[i], prev_velocities[i],
              prev_accelerations[i], joints_[i]->joint_->name.c_str());
  }

  // ------ Tacks on the new segments

  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;

  std::vector<double> durations(msg->points.size());
  durations[0] = msg->points[0].time_from_start.toSec();
  for (size_t i = 1; i < msg->points.size(); ++i)
    durations[i] = (msg->points[i].time_from_start - msg->points[i-1].time_from_start).toSec();

  // Checks if we should wrap
  std::vector<double> wrap(joints_.size(), 0.0);
  assert(!msg->points[0].positions.empty());
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    if (joints_[j]->joint_->type == urdf::Joint::CONTINUOUS)
    {
      double dist = angles::shortest_angular_distance(prev_positions[j], msg->points[0].positions[j]);
      wrap[j] = (prev_positions[j] + dist) - msg->points[0].positions[j];
    }
  }

  for (size_t i = 0; i < msg->points.size(); ++i)
  {
    Segment seg;

    seg.start_time = (msg->header.stamp + msg->points[i].time_from_start).toSec() - durations[i];
    seg.duration = durations[i];
    seg.splines.resize(joints_.size());

    // Checks that the incoming segment has the right number of elements.

    if (msg->points[i].accelerations.size() != 0 && msg->points[i].accelerations.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the accelerations", (int)i, (int)msg->points[i].accelerations.size());
      return;
    }
    if (msg->points[i].velocities.size() != 0 && msg->points[i].velocities.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the velocities", (int)i, (int)msg->points[i].velocities.size());
      return;
    }
    if (msg->points[i].positions.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the positions", (int)i, (int)msg->points[i].positions.size());
      return;
    }

    // Re-orders the joints in the command to match the interal joint order.

    accelerations.resize(msg->points[i].accelerations.size());
    velocities.resize(msg->points[i].velocities.size());
    positions.resize(msg->points[i].positions.size());
    for (size_t j = 0; j < joints_.size(); ++j)
    {
      if (!accelerations.empty()) accelerations[j] = msg->points[i].accelerations[lookup[j]];
      if (!velocities.empty()) velocities[j] = msg->points[i].velocities[lookup[j]];
      if (!positions.empty()) positions[j] = msg->points[i].positions[lookup[j]] + wrap[j];
    }

    // Converts the boundary conditions to splines.

    for (size_t j = 0; j < joints_.size(); ++j)
    {
      if (prev_accelerations.size() > 0 && accelerations.size() > 0)
      {
        getQuinticSplineCoefficients(
          prev_positions[j], prev_velocities[j], prev_accelerations[j],
          positions[j], velocities[j], accelerations[j],
          durations[i],
          seg.splines[j].coef);
      }
      else if (prev_velocities.size() > 0 && velocities.size() > 0)
      {
        getCubicSplineCoefficients(
          prev_positions[j], prev_velocities[j],
          positions[j], velocities[j],
          durations[i],
          seg.splines[j].coef);
        seg.splines[j].coef.resize(6, 0.0);
      }
      else
      {
        seg.splines[j].coef[0] = prev_positions[j];
        if (durations[i] == 0.0)
          seg.splines[j].coef[1] = 0.0;
        else
          seg.splines[j].coef[1] = (positions[j] - prev_positions[j]) / durations[i];
        seg.splines[j].coef[2] = 0.0;
        seg.splines[j].coef[3] = 0.0;
        seg.splines[j].coef[4] = 0.0;
        seg.splines[j].coef[5] = 0.0;
      }
    }

    // Pushes the splines onto the end of the new trajectory.

    new_traj.push_back(seg);

    // Computes the starting conditions for the next segment

    prev_positions = positions;
    prev_velocities = velocities;
    prev_accelerations = accelerations;
  }

  // ------ Commits the new trajectory

  if (!new_traj_ptr)
  {
    ROS_ERROR("The new trajectory was null!");
    return;
  }

  current_trajectory_box_.set(new_traj_ptr);
  ROS_DEBUG("The new trajectory has %d segments", (int)new_traj.size());
#if 0
  for (size_t i = 0; i < std::min((size_t)20,new_traj.size()); ++i)
  {
    ROS_DEBUG("Segment %2d: %.3lf for %.3lf", i, new_traj[i].start_time, new_traj[i].duration);
    for (size_t j = 0; j < new_traj[i].splines.size(); ++j)
    {
      ROS_DEBUG("    %.2lf  %.2lf  %.2lf  %.2lf , %.2lf  %.2lf(%s)",
                new_traj[i].splines[j].coef[0],
                new_traj[i].splines[j].coef[1],
                new_traj[i].splines[j].coef[2],
                new_traj[i].splines[j].coef[3],
                new_traj[i].splines[j].coef[4],
                new_traj[i].splines[j].coef[5],
                joints_[j]->joint_->name_.c_str());
    }
  }
#endif
}

bool JointSplineTrajectoryController::queryStateService(
  pr2_controllers_msgs::QueryTrajectoryState::Request &req,
  pr2_controllers_msgs::QueryTrajectoryState::Response &resp)
{
  boost::shared_ptr<const SpecifiedTrajectory> traj_ptr;
  current_trajectory_box_.get(traj_ptr);
  if (!traj_ptr)
  {
    ROS_FATAL("The current trajectory can never be null");
    return false;
  }
  const SpecifiedTrajectory &traj = *traj_ptr;

  // Determines which segment of the trajectory to use
  int seg = -1;
  while (seg + 1 < (int)traj.size() &&
         traj[seg+1].start_time < req.time.toSec())
  {
    ++seg;
  }
  if (seg == -1)
    return false;

  for (size_t i = 0; i < q.size(); ++i)
  {
  }


  resp.name.resize(joints_.size());
  resp.position.resize(joints_.size());
  resp.velocity.resize(joints_.size());
  resp.acceleration.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    resp.name[j] = joints_[j]->joint_->name;
    sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration,
                               req.time.toSec() - traj[seg].start_time,
                               resp.position[j], resp.velocity[j], resp.acceleration[j]);
  }

  return true;
}

void JointSplineTrajectoryController::sampleSplineWithTimeBounds(
  const std::vector<double>& coefficients, double duration, double time,
  double& position, double& velocity, double& acceleration)
{
  if (time < 0)
  {
    double _;
    sampleQuinticSpline(coefficients, 0.0, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else if (time > duration)
  {
    double _;
    sampleQuinticSpline(coefficients, duration, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else
  {
    sampleQuinticSpline(coefficients, time,
                        position, velocity, acceleration);
  }
}

}
