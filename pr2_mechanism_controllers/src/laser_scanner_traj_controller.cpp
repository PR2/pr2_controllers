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

#include "pr2_mechanism_controllers/laser_scanner_traj_controller.h"
#include <algorithm>
#include <cmath>
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(LaserScannerTrajController, controller::LaserScannerTrajController, pr2_controller_interface::Controller)
PLUGINLIB_REGISTER_CLASS(LaserScannerTrajControllerNode, controller::LaserScannerTrajControllerNode, pr2_controller_interface::Controller)

using namespace std ;
using namespace controller ;
using namespace filters ;

LaserScannerTrajController::LaserScannerTrajController() : traj_(1)
{
  tracking_offset_ = 0 ;
  //track_link_enabled_ = false ;
}

LaserScannerTrajController::~LaserScannerTrajController()
{

}

bool LaserScannerTrajController::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
  if (!robot || !config)
    return false ;
  robot_ = robot ;

  // ***** Name element *****
  const char* name = config->Attribute("name") ;
  if (!name)
  {
    ROS_ERROR("LaserScannerTrajController:: Name attribute not defined in controller tag") ;
    return false ;
  }
  name_ = name ;

  // ***** Joint Element *****
  TiXmlElement *j = config->FirstChildElement("joint") ;
  if (!j)
  {
    ROS_ERROR("%s:: joint element not defined inside controller", name_.c_str()) ;
    return false ;
  }
  const char *jn = j->Attribute("name") ;

  if (!jn)
  {
    ROS_ERROR("%s:: name attribute not defined insidejoint element", name_.c_str()) ;
    return false ;
  }
  joint_state_ = robot_->getJointState(string(jn)) ;  // Need joint state to check calibrated flag
  if (!joint_state_)
  {
    ROS_ERROR("%s:: Could not find joint \"%s\" in robot model", name_.c_str(), jn) ;
    return false ;
  }
  if (!joint_state_->joint_->limits)
  {
    ROS_ERROR("%s:: Joint \"%s\" has no limits specified", name_.c_str(), jn) ;
    return false ;
  }


  TiXmlElement *pid_elem = j->FirstChildElement("pid") ;
  if (!pid_elem)
  {
    ROS_ERROR("%s:: Could not find element \"pid\" in joint", name_.c_str()) ;
    return false ;
  }

  bool result ;
  result = pid_controller_.initXml(pid_elem) ;
  if (!result)
  {
    ROS_ERROR("%s:: Error initializing pid element", name_.c_str()) ;
    return false ;
  }
  last_time_ = robot->getTime() ;
  last_error_ = 0.0 ;

  // ***** Derivative Error Filter Element *****
  TiXmlElement *filter_elem = config->FirstChildElement("filter");
  if(!filter_elem)
  {
    ROS_ERROR("%s:: filter element not defined inside controller", name_.c_str()) ;
    return false ;
  }

  std::string xml_string;
  TiXmlElement *struct_elem = filter_elem->FirstChildElement("value");
  if(!struct_elem)
  {
    ROS_ERROR("Xml is missing a <value> tag inside filter spec, cannot parse!");
    return false;
  }

  TiXmlPrinter printer;
  printer.SetIndent("  ");
  struct_elem->Accept(&printer);
  std::string filter_str = printer.Str();
  ROS_DEBUG("Constructing filter with XML: %s", filter_str.c_str());
  //xmlrpc_elem->Print(xml_string, 10);
  int offset = 0;
  XmlRpc::XmlRpcValue rpc_config(filter_str, &offset);
  ROS_DEBUG("XmlRpc parsed xml: %s type: %d", rpc_config.toXml().c_str(), rpc_config.getType());
  d_error_filter.MultiChannelFilterBase<double>::configure((unsigned int)1, rpc_config) ;

  // ***** Max Rate and Acceleration Elements *****
  TiXmlElement *max_rate_elem = config->FirstChildElement("max_rate") ;
  if (!max_rate_elem)
    return false ;
  if(max_rate_elem->QueryDoubleAttribute("value", &max_rate_) != TIXML_SUCCESS )
    return false ;

  TiXmlElement *max_acc_elem = config->FirstChildElement("max_acc") ;
  if (!max_acc_elem)
    return false ;
  if(max_acc_elem->QueryDoubleAttribute("value", &max_acc_) != TIXML_SUCCESS )
    return false ;

  // Set to hold the current position

  pr2_msgs::PeriodicCmd cmd ;
  cmd.profile = "linear" ;
  cmd.period = 1.0 ;
  cmd.amplitude = 0.0 ;
  cmd.offset = joint_state_->position_ ;

  setPeriodicCmd(cmd) ;


  return true ;
}

bool LaserScannerTrajController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  std::string xml;
  if (!n.getParam("xml", xml))
  {
    ROS_ERROR("No XML given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  TiXmlDocument doc;
  doc.Parse(xml.c_str());
  if (!doc.RootElement())
  {
    ROS_ERROR("Error parsing XML (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  return initXml(robot, doc.RootElement());
}

void LaserScannerTrajController::update()
{
  if (!joint_state_->calibrated_)
    return;

  // ***** Compute the offset from tracking a link *****
  //! \todo replace this link tracker with a KDL inverse kinematics solver
  /*if(track_link_lock_.try_lock())
  {
    if (track_link_enabled_  && target_link_ && mount_link_)
    {
      // Compute the position of track_point_ in the world frame
      tf::Pose link_pose(target_link_->abs_orientation_, target_link_->abs_position_) ;
      tf::Point link_point_world ;
      link_point_world = link_pose*track_point_ ;

      // We're hugely approximating our inverse kinematics. This is probably good enough for now...
      double dx = link_point_world.x() - mount_link_->abs_position_.x() ;
      double dz = link_point_world.z() - mount_link_->abs_position_.z() ;
      tracking_offset_ = atan2(-dz,dx) ;
    }
    else
    {
      tracking_offset_ = 0.0 ;
    }
    track_link_lock_.unlock() ;
    }*/

  tracking_offset_ = 0.0 ;
  // ***** Compute the current command from the trajectory profile *****
  if (traj_lock_.try_lock())
  {
    if (traj_duration_ > 1e-6)                                   // Short trajectories could make the mod_time calculation unstable
    {
      double profile_time = getCurProfileTime() ;

      trajectory::Trajectory::TPoint sampled_point ;
      sampled_point.dimension_ = 1 ;
      sampled_point.q_.resize(1) ;
      sampled_point.qdot_.resize(1) ;
      int result ;

      result = traj_.sample(sampled_point, profile_time) ;
      if (result > 0)
        traj_command_ = sampled_point.q_[0] ;
    }
    traj_lock_.unlock() ;
  }

  // ***** Run the position control loop *****
  double cmd = traj_command_ + tracking_offset_ ;

  ros::Time time = robot_->getTime();
  double error(0.0) ;
  angles::shortest_angular_distance_with_limits(cmd, joint_state_->position_,
                                                joint_state_->joint_->limits->lower,
                                                joint_state_->joint_->limits->upper,
                                                error) ;
  ros::Duration dt = time - last_time_ ;
  double d_error = (error-last_error_)/dt.toSec();
  std::vector<double> vin;
  vin.push_back(d_error);
  std::vector<double> vout = vin;

  d_error_filter.update(vin, vout) ;

  double filtered_d_error  = vout[0];

  // Add filtering step
  // Update pid with d_error added
  joint_state_->commanded_effort_ = pid_controller_.updatePid(error, filtered_d_error, dt) ;
  last_time_ = time ;
  last_error_ = error ;
}

double LaserScannerTrajController::getCurProfileTime()
{
  ros::Time time = robot_->getTime();
  double time_from_start = (time - traj_start_time_).toSec();
  double mod_time = time_from_start - floor(time_from_start/traj_.getTotalTime())*traj_.getTotalTime() ;
  return mod_time ;
}

double LaserScannerTrajController::getProfileDuration()
{
  return traj_duration_ ;
}

int LaserScannerTrajController::getCurProfileSegment()
{
  double cur_time = getCurProfileTime() ;
  return traj_.findTrajectorySegment(cur_time) ;
}

bool LaserScannerTrajController::setTrajectory(const std::vector<trajectory::Trajectory::TPoint>& traj_points, double max_rate, double max_acc, std::string interp)
{
  while (!traj_lock_.try_lock())
    usleep(100) ;

  vector<double> max_rates ;
  max_rates.push_back(max_rate) ;
  vector<double> max_accs ;
  max_accs.push_back(max_acc) ;


  traj_.autocalc_timing_ = true;

  traj_.setMaxRates(max_rates) ;
  traj_.setMaxAcc(max_accs) ;
  traj_.setInterpolationMethod(interp) ;

  traj_.setTrajectory(traj_points) ;

  traj_start_time_ = robot_->getTime() ;

  traj_duration_ = traj_.getTotalTime() ;

  traj_lock_.unlock() ;

  return true;
}

bool LaserScannerTrajController::setPeriodicCmd(const pr2_msgs::PeriodicCmd& cmd)
{
  if (cmd.profile == "linear" ||
      cmd.profile == "blended_linear")
  {
    double high_pt = cmd.amplitude + cmd.offset ;
    double low_pt = -cmd.amplitude + cmd.offset ;


    double soft_limit_low  = joint_state_->joint_->limits->lower;
    double soft_limit_high = joint_state_->joint_->limits->upper;

    if (low_pt < soft_limit_low)
    {
      ROS_WARN("Lower setpoint (%.3f) is below the soft limit (%.3f). Truncating command", low_pt, soft_limit_low) ;
      low_pt = soft_limit_low ;
    }

    if (high_pt > soft_limit_high)
    {
      ROS_WARN("Upper setpoint (%.3f) is above the soft limit (%.3f). Truncating command", high_pt, soft_limit_high) ;
      high_pt = soft_limit_high ;
    }

    std::vector<trajectory::Trajectory::TPoint> tpoints ;

    trajectory::Trajectory::TPoint cur_point(1) ;

    cur_point.dimension_ = 1 ;

    cur_point.q_[0] = low_pt ;
    cur_point.time_ = 0.0 ;
    tpoints.push_back(cur_point) ;

    cur_point.q_[0] = high_pt ;
    cur_point.time_ = cmd.period/2.0 ;
    tpoints.push_back(cur_point) ;

    cur_point.q_[0] = low_pt ;
    cur_point.time_ = cmd.period ;
    tpoints.push_back(cur_point) ;

    if (!setTrajectory(tpoints, max_rate_, max_acc_, cmd.profile)){
      ROS_ERROR("Failed to set tilt laser scanner trajectory.") ;
      return false;
    }
    else{
      ROS_INFO("LaserScannerTrajController: Periodic Command set. Duration=%.4f sec", getProfileDuration()) ;
      return true;
    }
  }
  else
  {
    ROS_WARN("Unknown Periodic Trajectory Type. Not setting command.") ;
    return false;
  }
}

bool LaserScannerTrajController::setTrajCmd(const pr2_msgs::LaserTrajCmd& traj_cmd)
{
  if (traj_cmd.profile == "linear" ||
      traj_cmd.profile == "blended_linear")
  {
    const unsigned int N = traj_cmd.pos.size() ;
    if (traj_cmd.time.size() != N)
    {
      ROS_ERROR("# Times and # Pos must match! pos.size()=%u times.size()=%u", N, traj_cmd.time.size()) ;
      return false ;
    }

    // Load up the trajectory data points, 1 point at a time
    std::vector<trajectory::Trajectory::TPoint> tpoints ;
    for (unsigned int i=0; i<N; i++)
    {
      trajectory::Trajectory::TPoint cur_point(1) ;
      cur_point.dimension_ = 1 ;
      cur_point.q_[0] = traj_cmd.pos[i] ;
      cur_point.time_ = traj_cmd.time[i] ;
      tpoints.push_back(cur_point) ;
    }

    double cur_max_rate = max_rate_ ;
    double cur_max_acc  = max_acc_ ;

    // Overwrite our limits, if they're specified in the msg. Is this maybe too dangerous?
    if (traj_cmd.max_rate > 0)                  // Only overwrite if a positive val
      cur_max_rate = traj_cmd.max_rate ;
    if (traj_cmd.max_accel > 0)
      cur_max_acc = traj_cmd.max_accel ;

    if (!setTrajectory(tpoints, cur_max_rate, cur_max_acc, traj_cmd.profile))
    {
      ROS_ERROR("Failed to set tilt laser scanner trajectory.") ;
      return false;
    }
    else
    {
      ROS_INFO("LaserScannerTrajController: Trajectory Command set. Duration=%.4f sec", getProfileDuration()) ;
      return true;
    }
  }
  else
  {
    ROS_WARN("Unknown Periodic Trajectory Type. Not setting command.") ;
    return false;
  }
}

/*bool LaserScannerTrajController::setTrackLinkCmd(const pr2_mechanism_controllers::TrackLinkCmd& track_link_cmd)
{
  while (!track_link_lock_.try_lock())
    usleep(100) ;

  if (track_link_cmd.enable)
  {
    ROS_INFO("LaserScannerTrajController:: Tracking link %s", track_link_cmd.link_name.c_str()) ;
    track_link_enabled_ = true ;
    string mount_link_name = "laser_tilt_mount_link" ;
    target_link_ = robot_->getLinkState(track_link_cmd.link_name) ;
    mount_link_  = robot_->getLinkState(mount_link_name) ;
    tf::pointMsgToTF(track_link_cmd.point, track_point_) ;

    if (target_link_ == NULL)
    {
      ROS_ERROR("LaserScannerTrajController:: Could not find target link:%s", track_link_cmd.link_name.c_str()) ;
      track_link_enabled_ = false ;
    }
    if (mount_link_ == NULL)
    {
      ROS_ERROR("LaserScannerTrajController:: Could not find mount link:%s", mount_link_name.c_str()) ;
      track_link_enabled_ = false ;
    }
  }
  else
  {
    track_link_enabled_ = false ;
    ROS_INFO("LaserScannerTrajController:: No longer tracking link") ;
  }

  track_link_lock_.unlock() ;

  return track_link_enabled_;
  }*/


LaserScannerTrajControllerNode::LaserScannerTrajControllerNode(): c_()
{
  prev_profile_segment_ = -1 ;
  need_to_send_msg_ = false ;                                           // Haven't completed a sweep yet, so don't need to send a msg
  publisher_ = NULL ;                                                   // We don't know our topic yet, so we can't build it
}

LaserScannerTrajControllerNode::~LaserScannerTrajControllerNode()
{
  publisher_->stop() ;

  delete publisher_ ;    // Probably should wait on publish_->is_running() before exiting. Need to
                         //   look into shutdown semantics for realtime_publisher
}

void LaserScannerTrajControllerNode::update()
{
  c_.update() ;

  int cur_profile_segment = c_.getCurProfileSegment() ;

  if (cur_profile_segment != prev_profile_segment_)
  {
    // Should we be populating header.stamp here? Or, we can simply let ros take care of the timestamp
    ros::Time cur_time(robot_->getTime()) ;
    m_scanner_signal_.header.stamp = cur_time ;
    m_scanner_signal_.signal = cur_profile_segment ;
    need_to_send_msg_ = true ;
  }

  prev_profile_segment_ = cur_profile_segment ;

  // Use the realtime_publisher to try to send the message.
  //   If it fails sending, it's not a big deal, since we can just try again 1 ms later. No one will notice.
  if (need_to_send_msg_)
  {
    if (publisher_->trylock())
    {
      publisher_->msg_.header = m_scanner_signal_.header ;
      publisher_->msg_.signal = m_scanner_signal_.signal ;
      publisher_->unlockAndPublish() ;
      need_to_send_msg_ = false ;
    }
  }
}

bool LaserScannerTrajControllerNode::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
  robot_ = robot ;      // Need robot in order to grab hardware time

  service_prefix_ = config->Attribute("name") ;
  node_ = ros::NodeHandle(service_prefix_);

  if (!c_.initXml(robot, config))
  {
    ROS_ERROR("Error Loading LaserScannerTrajControllerNode XML") ;
    return false ;
  }

  sub_set_periodic_cmd_ =
    node_.subscribe("set_periodic_cmd", 1, &LaserScannerTrajControllerNode::setPeriodicCmd, this) ;
  sub_set_traj_cmd_ =
    node_.subscribe("set_traj_cmd", 1, &LaserScannerTrajControllerNode::setTrajCmd, this) ;

  serve_set_periodic_cmd_ =
    node_.advertiseService("set_periodic_cmd", &LaserScannerTrajControllerNode::setPeriodicSrv, this);
  serve_set_Traj_cmd_ =
    node_.advertiseService("set_traj_cmd", &LaserScannerTrajControllerNode::setTrajSrv, this);

  if (publisher_ != NULL)               // Make sure that we don't memory leak if initXml gets called twice
    delete publisher_ ;
  publisher_ = new realtime_tools::RealtimePublisher <pr2_msgs::LaserScannerSignal> (node_, "laser_scanner_signal", 1) ;

  prev_profile_segment_ = -1 ;

  ROS_INFO("Successfully spawned %s", service_prefix_.c_str()) ;

  return true ;
}

bool LaserScannerTrajControllerNode::init(pr2_mechanism_model::RobotState *robot,
                                          ros::NodeHandle &n)
{
  std::string xml;
  if (!n.getParam("xml", xml))
  {
    ROS_ERROR("No XML given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  TiXmlDocument doc;
  doc.Parse(xml.c_str());
  if (!doc.RootElement())
  {
    ROS_ERROR("Error parsing XML (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  return initXml(robot, doc.RootElement());
}


bool LaserScannerTrajControllerNode::setPeriodicSrv(pr2_msgs::SetPeriodicCmd::Request &req,
                                                    pr2_msgs::SetPeriodicCmd::Response &res)
{
  ROS_INFO("LaserScannerTrajControllerNode: set periodic command");

  if (!c_.setPeriodicCmd(req.command))
    return false;
  else
  {
    res.start_time = ros::Time::now();
    prev_profile_segment_ = -1 ;
    return true;
  }
}

void LaserScannerTrajControllerNode::setPeriodicCmd(const pr2_msgs::PeriodicCmdConstPtr &cmd)
{
  c_.setPeriodicCmd(*cmd) ;
  prev_profile_segment_ = -1 ;
}

bool LaserScannerTrajControllerNode::setTrajSrv(pr2_msgs::SetLaserTrajCmd::Request &req,
                                                pr2_msgs::SetLaserTrajCmd::Response &res)
{
  ROS_INFO("LaserScannerTrajControllerNode: set traj command");

  if (!c_.setTrajCmd(req.command))
    return false;
  else
  {
    res.start_time = ros::Time::now();
    prev_profile_segment_ = -1 ;
    return true;
  }
}

void LaserScannerTrajControllerNode::setTrajCmd(const pr2_msgs::LaserTrajCmdConstPtr &traj_cmd)
{
  c_.setTrajCmd(*traj_cmd) ;
  prev_profile_segment_ = -1 ;
}

/*void LaserScannerTrajControllerNode::setTrackLinkCmd()
{
  c_.setTrackLinkCmd(track_link_cmd_) ;
  }*/

