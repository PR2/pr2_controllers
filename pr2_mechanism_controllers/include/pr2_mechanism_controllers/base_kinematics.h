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
/*
 * Author: Sachin Chitta and Matthew Piccoli
 */

#ifndef PR2_BASE_KINEMATICS_H
#define PR2_BASE_KINEMATICS_H

#include <pr2_mechanism_model/robot.h>
#include <pr2_controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <control_toolbox/filters.h>
#include <boost/thread/condition.hpp>

namespace controller
{
  class Wheel;
  class Caster;
  class BaseKinematics;
  /*! \class
   \brief This class keeps track of the wheels
   */
  class Wheel
  {
    public:

      /*!
       * \brief JointState for this wheel joint
       */
      pr2_mechanism_model::JointState *joint_;

      /*!
       * \brief default offset from the parent caster before any transformations
       */
      geometry_msgs::Point offset_;

      /*!
       * \brief name of the joint
       */
      std::string joint_name_;

      /*!
       * \brief name of the link
       */
      std::string link_name_;


      /*!
       * \brief offset_ after rotation transformation from the parent caster's position
       */
      geometry_msgs::Point position_;

      /*!
       * \brief the caster on which this wheel is mounted
       */
      Caster *parent_;

      /*!
       * \brief actual wheel speeds
       */
      double wheel_speed_actual_;

      /*!
       * \brief desired wheel speed
       */
      double wheel_speed_cmd_;

      /*!
       * \brief difference between desired and actual speed
       */
      double wheel_speed_error_;

      /*!
       * \brief wheel speed filtered with alpha
       */
      double wheel_speed_filtered_;

      /*!
       * \brief specifies the default direction of the wheel
       */
      int direction_multiplier_;

      /*!
       * \brief remembers if the wheel is stalled
       */
      int wheel_stuck_;

      /*!
       * \brief wheel radius scale (based on the default wheel radius in Basekinematics)
       */
      double wheel_radius_;

      /*!
       * \brief Loads wheel's information from the xml description file and param server
       * @param robot_state The robot's current state
       * @param config Tiny xml element pointing to this wheel
       */
      bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node, std::string link_name);

      /*!
       * \brief Computes 2d postion of the wheel relative to the center of the base
       */
      void updatePosition();
  };

  /*! \class
   \brief This class keeps track of the casters
   */
  class Caster
  {
    public:

      /*!
       * \brief JointState for this caster joint
       */
      pr2_mechanism_model::JointState *joint_;

      /*!
       * \brief offset from the center of the base
       */
      geometry_msgs::Point offset_;

      /*!
       * \brief name of the link
       */
      std::string link_name_;

      /*!
       * \brief name of the joint
       */
      std::string joint_name_;

      //geometry_msgs::Point position_;

      /*!
       * \brief BaseKinematics to which this caster belongs
       */
      BaseKinematics *parent_;

      /*!
       * \brief the number of child wheels that are attached to this caster
       */
      int num_children_;

      /*!
       * \brief actual caster steer angles
       */
      double steer_angle_actual_;

      /*!
       * \brief actual caster steer angles
       */
      double steer_angle_desired_;

      /*!
       * \brief vector of desired caster steer speeds
       */
      double steer_velocity_desired_;

      /*!
       * \brief stored caster steer angles
       */
      double steer_angle_stored_;

      /*!
       * \brief difference between desired and actual angles of the casters
       */
      double caster_position_error_;

      /*!
       * \brief difference between desired and actual speeds of the casters
       */
      double caster_speed_error_;

      /*!
       * \brief caster speed filtered with alpha
       */
      double caster_speed_filtered_;

      /*!
       * \brief remembers the caster's current speed
       */
      double caster_speed_;

      /*!
       * \brief remembers if the caster is stalled
       */
      int caster_stuck_;

      bool init(pr2_mechanism_model::RobotState *robot_state,  ros::NodeHandle &node, std::string link_name);
  };

  /*! \class
   \brief This class includes common functions used by the base controller and odometry
   */
  class BaseKinematics
  {
    public:

      /*!
       * \brief Loads BaseKinematic's information from the xml description file and param server
       * @param robot_state The robot's current state
       * @param config Tiny xml element pointing to its controller
       * @return Successful init
       */
    bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node);

      /*!
       * \brief Computes 2d postion of every wheel relative to the center of the base
       */
      void computeWheelPositions();

      /*!
       * \brief Computes 2d velocity of a point at relative distance pos to another point with velocity (and rotation (z)) vel
       * @param pos The position of the object relative to the center of rotation
       * @param vel Velocity of the center of rotation
       * @return Velocity at the given point
       */
      geometry_msgs::Twist pointVel2D(const geometry_msgs::Point& pos, const geometry_msgs::Twist& vel);

      /*!
       * \brief remembers everything about the state of the robot
       */
      pr2_mechanism_model::RobotState *robot_state_;

      /*!
       * \brief number of wheels connected to the base
       */
      int num_wheels_;

      /*!
       * \brief number of casters connected to the base
       */
      int num_casters_;

      /*!
       * \brief vector of every wheel attached to the base
       */
      std::vector<Wheel> wheel_;

      /*!
       * \brief vector of every caster attached to the base
       */
      std::vector<Caster> caster_;

      /*!
       * \brief the name of the casters in the xml file
       */
      std::string xml_caster_name_;

      /*!
       * \brief the name of the wheels in the xml file
       */
      std::string xml_wheel_name_;

      /*!
       * \brief name of this BaseKinematics (generally associated with whatever controller is using it)
       */
      std::string name_;

      /*!
       * \brief maximum dT used in computation of interpolated velocity command
       */
      double MAX_DT_;

      /*!
       * \brief Name(string id) of the robot base frame
       */
      std::string robot_base_id_;
  };
} // namespace
#endif
