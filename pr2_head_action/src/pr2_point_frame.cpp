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

#include <ros/ros.h>

#include <vector>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include <actionlib/server/action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include "tf_conversions/tf_kdl.h"
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

#include <urdf/model.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>


void printVector3(const char * label, tf::Vector3 v)
{
  printf("%s % 7.3f % 7.3f % 7.3f\n", label, v.x(), v.y(), v.z() );

}

class ControlHead
{
private:
  typedef actionlib::ActionServer<pr2_controllers_msgs::PointHeadAction> PHAS;
  typedef PHAS::GoalHandle GoalHandle;

  std::string node_name_;
  std::string action_name_;
  std::string root_;
  std::string tip_;
  std::string pan_link_;
  std::string default_pointing_frame_;
  std::string pointing_frame_;
  tf::Vector3 pointing_axis_;
  std::vector< std::string> joint_names_;

  ros::NodeHandle nh_, pnh_;
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

  geometry_msgs::PointStamped target_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  tf::Point target_in_root_;

  boost::scoped_ptr<KDL::ChainFkSolverPos> pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  tf::TransformListener tfl_;
  urdf::Model urdf_model_;

public:

  ControlHead(const ros::NodeHandle &node) :
      node_name_(ros::this_node::getName())
      , action_name_("point_head_action")
      , nh_(node)
      , pnh_("~")
      , action_server_(nh_, action_name_.c_str(),
                         boost::bind(&ControlHead::goalCB, this, _1),
                         boost::bind(&ControlHead::cancelCB, this, _1), false)
      , has_active_goal_(false)
  {
    pnh_.param("pan_link", pan_link_, std::string("head_pan_link"));
    pnh_.param("default_pointing_frame", default_pointing_frame_, std::string("head_tilt_link"));
    pnh_.param("success_angle_threshold", success_angle_threshold_, 0.1);

    if(pan_link_[0] == '/') pan_link_.erase(0, 1);
    if(default_pointing_frame_[0] == '/') default_pointing_frame_.erase(0, 1);

    // Connects to the controller
    pub_controller_command_ =
      nh_.advertise<trajectory_msgs::JointTrajectory>("command", 2);
    sub_controller_state_ =
      nh_.subscribe("state", 1, &ControlHead::controllerStateCB, this);
    cli_query_traj_ =
        nh_.serviceClient<pr2_controllers_msgs::QueryTrajectoryState>("/head_traj_controller/query_state");

    // Should only ever happen on first call... move to constructor?
    if(tree_.getNrOfJoints() == 0)
    {
      std::string robot_desc_string;
      nh_.param("/robot_description", robot_desc_string, std::string());
      ROS_DEBUG("Reading tree from robot_description...");
      if (!kdl_parser::treeFromString(robot_desc_string, tree_)){
         ROS_ERROR("Failed to construct kdl tree");
         exit(-1);
      }
      if (!urdf_model_.initString(robot_desc_string)){
        ROS_ERROR("Failed to parse urdf string for urdf::Model.");
        exit(-2);
      }
    }

    ROS_DEBUG("Tree has %d joints and %d segments.", tree_.getNrOfJoints(), tree_.getNrOfSegments());

    action_server_.start();

    watchdog_timer_ = nh_.createTimer(ros::Duration(1.0), &ControlHead::watchdog, this);
  }

  void goalCB(GoalHandle gh)
  {
    // Before we do anything, we need to know that name of the pan_link's parent, which we will treat as the root.
    if (root_.empty())
    {
      for (int i = 0; i < 10; ++i)
      {
        try {
          tfl_.getParent(pan_link_, ros::Time(), root_);
          break;
        }
        catch (const tf::TransformException &ex) {}
        ros::Duration(0.5).sleep();
      }
      if (root_.empty())
      {
        ROS_ERROR("Could not get parent of %s in the TF tree", pan_link_.c_str());
        gh.setRejected();
        return;
      }
    }
    if(root_[0] == '/') root_.erase(0, 1);

    ROS_DEBUG("Got point head goal!");

    // Process pointing frame and axis
    const geometry_msgs::PointStamped &target = gh.getGoal()->target;
    pointing_frame_ = gh.getGoal()->pointing_frame;
    if(pointing_frame_.length() == 0)
    {
      ROS_WARN("Pointing frame not specified, using %s [1, 0, 0] by default.", default_pointing_frame_.c_str());
      pointing_frame_ = default_pointing_frame_;
      pointing_axis_ = tf::Vector3(1.0, 0.0, 0.0);
    }
    else
    {
      if(pointing_frame_[0] == '/') pointing_frame_.erase(0, 1);
      bool ret1 = false;
      try
      {
        std::string error_msg;
        ret1 = tfl_.waitForTransform(pan_link_, pointing_frame_, target.header.stamp,
                                   ros::Duration(5.0), ros::Duration(0.01), &error_msg);

        tf::vector3MsgToTF(gh.getGoal()->pointing_axis, pointing_axis_);
        if(pointing_axis_.length() < 0.1)
        {
          size_t found = pointing_frame_.find("optical_frame");
          if (found != std::string::npos)
          {
            ROS_WARN("Pointing axis appears to be zero-length. Using [0, 0, 1] as default for an optical frame.");
            pointing_axis_ = tf::Vector3(0, 0, 1);
          }
          else
          {
            ROS_WARN("Pointing axis appears to be zero-length. Using [1, 0, 0] as default for a non-optical frame.");
            pointing_axis_ = tf::Vector3(1, 0, 0);
          }
        }
        else
        {
          pointing_axis_.normalize();
        }
      }
      catch(const tf::TransformException &ex)
      {
        ROS_ERROR("Transform failure (%d): %s", ret1, ex.what());
        gh.setRejected();
        return;
      }
    }

    //Put the target point in the root frame (usually torso_lift_link).
    bool ret1 = false;
    try
    {
      std::string error_msg;
      ret1 = tfl_.waitForTransform(root_.c_str(), target.header.frame_id, target.header.stamp,
                                       ros::Duration(5.0), ros::Duration(0.01), &error_msg);

      geometry_msgs::PointStamped target_in_root_msg;
      tfl_.transformPoint(root_.c_str(), target, target_in_root_msg );
      tf::pointMsgToTF(target_in_root_msg.point, target_in_root_);
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR("Transform failure (%d): %s", ret1, ex.what());
      gh.setRejected();
      return;
    }

    if( tip_.compare(pointing_frame_) != 0 )
    {
      bool success = tree_.getChain(root_.c_str(), pointing_frame_.c_str(), chain_);
      if( !success )
      {
        ROS_ERROR("Couldn't create chain from %s to %s.", root_.c_str(), pointing_frame_.c_str());
        gh.setRejected();
        return;
      }
      tip_ = pointing_frame_;

      pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
      jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
      joint_names_.resize(chain_.getNrOfJoints());
    }

    unsigned int joints = chain_.getNrOfJoints();

//    int segments = chain_.getNrOfSegments();
//    ROS_INFO("Parsed urdf from %s to %s and found %d joints and %d segments.", root_.c_str(), pointing_frame_.c_str(), joints, segments);
//    for(int i = 0; i < segments; i++)
//    {
//      KDL::Segment segment = chain_.getSegment(i);
//      ROS_INFO("Segment %d, %s: joint %s type %s",
//               i, segment.getName().c_str(), segment.getJoint().getName().c_str(), segment.getJoint().getTypeName().c_str() );
//    }

    KDL::JntArray jnt_pos(joints), jnt_eff(joints);
    KDL::Jacobian jacobian(joints);

    pr2_controllers_msgs::QueryTrajectoryState traj_state;
    traj_state.request.time = ros::Time::now() + ros::Duration(0.01);
    if (!cli_query_traj_.call(traj_state))
    {
      ROS_ERROR("Service call to query controller trajectory failed.");
      gh.setRejected();
      return;
    }
    if(traj_state.response.name.size() != joints)
    {
      ROS_ERROR("Number of joints mismatch: urdf chain vs. trajectory controller state.");
      gh.setRejected();
      return;
    }
    std::vector<urdf::JointLimits> limits_(joints);

    // Get initial joint positions and joint limits.
    for(unsigned int i = 0; i < joints; i++)
    {
      joint_names_[i] = traj_state.response.name[i];
      limits_[i] = *(urdf_model_.joints_[joint_names_[i].c_str()]->limits);
      ROS_DEBUG("Joint %d %s: %f, limits: %f %f", i, traj_state.response.name[i].c_str(), traj_state.response.position[i], limits_[i].lower, limits_[i].upper);
      //jnt_pos(i) = traj_state.response.position[i];
      jnt_pos(i) = 0;
    }

    int count = 0;
    int limit_flips = 0;
    float correction_angle = 2*M_PI;
    float correction_delta = 2*M_PI;
    while( ros::ok() && fabs(correction_delta) > 0.001)
    {
      //get the pose and jacobian for the current joint positions
      KDL::Frame pose;
      pose_solver_->JntToCart(jnt_pos, pose);
      jac_solver_->JntToJac(jnt_pos, jacobian);

      tf::Transform frame_in_root;
      tf::poseKDLToTF(pose, frame_in_root);

      tf::Vector3 axis_in_frame = pointing_axis_.normalized();
      tf::Vector3 target_from_frame = (target_in_root_ - frame_in_root.getOrigin()).normalized();
      tf::Vector3 current_in_frame = frame_in_root.getBasis().inverse()*target_from_frame;
      float prev_correction = correction_angle;
      correction_angle = current_in_frame.angle(axis_in_frame);
      correction_delta = correction_angle - prev_correction;
      ROS_DEBUG("At step %d, joint poses are %.4f and %.4f, angle error is %f", count, jnt_pos(0), jnt_pos(1), correction_angle);
      if(correction_angle < 0.5*success_angle_threshold_) break;
      tf::Vector3 correction_axis = frame_in_root.getBasis()*(axis_in_frame.cross(current_in_frame).normalized());
      //printVector3("correction_axis in root:", correction_axis);
      tf::Transform correction_tf(tf::Quaternion(correction_axis, 0.5*correction_angle), tf::Vector3(0,0,0));
      KDL::Frame correction_kdl;
      tf::transformTFToKDL(correction_tf, correction_kdl);

      // We apply a "wrench" proportional to the desired correction
      KDL::Frame identity_kdl;
      KDL::Twist twist = diff(correction_kdl, identity_kdl);
      KDL::Wrench wrench_desi;
      for (unsigned int i=0; i<6; i++)
        wrench_desi(i) = -1.0*twist(i);

      // Converts the "wrench" into "joint corrections" with a jacbobian-transpose
      for (unsigned int i = 0; i < joints; i++)
      {
        jnt_eff(i) = 0;
        for (unsigned int j=0; j<6; j++)
          jnt_eff(i) += (jacobian(j,i) * wrench_desi(j));
        jnt_pos(i) += jnt_eff(i);
      }

      // account for pan_link joint limit in back.
      if(jnt_pos(0) < limits_[0].lower && limit_flips++ == 0){ jnt_pos(0) += 1.5*M_PI; }
      if(jnt_pos(0) > limits_[0].upper && limit_flips++ == 0){ jnt_pos(0) -= 1.5*M_PI; }

      jnt_pos(1) = std::max(limits_[1].lower, jnt_pos(1));
      jnt_pos(1) = std::min(limits_[1].upper, jnt_pos(1));

      count++;

      if(limit_flips > 1){
        ROS_ERROR("Goal is out of joint limits, trying to point there anyway... \n");
        break;
      }
    }
    ROS_DEBUG("Iterative solver took %d steps", count);
    
    std::vector<double> q_goal(joints);

    for(unsigned int i = 0; i < joints; i++)
    {
      jnt_pos(i) = std::max(limits_[i].lower, jnt_pos(i));
      jnt_pos(i) = std::min(limits_[i].upper, jnt_pos(i));
      q_goal[i] = jnt_pos(i);
      ROS_DEBUG("Joint %d %s: %f", i, joint_names_[i].c_str(), jnt_pos(i));
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

    // Determines if we need to increase the duration of the movement in order to enforce a maximum velocity.
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

    traj.joint_names.push_back(traj_state.response.name[0]);
    traj.joint_names.push_back(traj_state.response.name[1]);

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
      KDL::JntArray jnt_pos(msg->joint_names.size());
      for(unsigned int i = 0; i < msg->joint_names.size(); i++)
        jnt_pos(i) = msg->actual.positions[i];

      KDL::Frame pose;
      pose_solver_->JntToCart(jnt_pos, pose);

      tf::Transform frame_in_root;
      tf::poseKDLToTF(pose, frame_in_root);

      tf::Vector3 axis_in_frame = pointing_axis_.normalized();
      tf::Vector3 target_from_frame = target_in_root_ - frame_in_root.getOrigin();

      target_from_frame.normalize();
      tf::Vector3 current_in_frame = frame_in_root.getBasis().inverse()*target_from_frame;

      pr2_controllers_msgs::PointHeadFeedback feedback;
      feedback.pointing_angle_error = current_in_frame.angle(axis_in_frame);
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

