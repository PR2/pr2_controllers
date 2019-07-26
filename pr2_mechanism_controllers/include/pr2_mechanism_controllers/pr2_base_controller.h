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

#include <ros/node_handle.h>
#include <realtime_tools/realtime_publisher.h>
#include <pr2_mechanism_controllers/BaseControllerState.h>
#include <robot_mechanism_controllers/joint_velocity_controller.h>
#include <pr2_mechanism_controllers/base_kinematics.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <filters/transfer_function.h>

namespace controller
{
  /*! \class
   \brief This class inherits from Controller and implements the actual controls.
   */
  class Pr2BaseController: public pr2_controller_interface::Controller
  {
    public:
      /*!
       * \brief Default Constructor of the Pr2BaseController class
       *
       */
      Pr2BaseController();

      /*!
       * \brief Destructor of the Pr2BaseController class
       *
       */
      ~Pr2BaseController();

      /*
       * \brief  The starting method is called by the realtime thread just before
       * the first call to update.  Overrides Controller.staring().
       * @return Successful start
       */
      void starting();

      bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

      /*
       * \brief callback function for setting the desired velocity using a topic
       * @param cmd_vel Velocity command of the base in m/s and rad/s
       */
      void setCommand(const geometry_msgs::Twist &cmd_vel);

      /*!
       * \brief Returns the current position command
       * @return Current velocity command
       */
      geometry_msgs::Twist getCommand();

      /*!
       * \brief class where the robot's information is computed and stored
       * @return BaseKinematic instance that is being used by this controller
       */
      BaseKinematics base_kin_;

      /*!
       * \brief mutex lock for setting and getting commands
       */
      pthread_mutex_t pr2_base_controller_lock_;

      /*!
       * \brief (a) Updates commands to caster and wheels.
       * Called every timestep in realtime
       */
      void update();

      /*!
       * \brief set the joint commands
       */
      void setJointCommands();

      /*!
       * \brief set the desired caster steer
       */
      void setDesiredCasterSteer();

    private:

      ros::NodeHandle node_;

      ros::NodeHandle root_handle_;

      ros::Subscriber cmd_sub_;

      ros::Subscriber cmd_sub_deprecated_;

      /*!
       * \brief timeout specifying time that the controller waits before setting the current velocity command to zero
       */
      double timeout_;

      /*!
       * \brief true when new command received by node
       */
      bool new_cmd_available_;

      /*!
       * \brief time corresponding to when update was last called
       */
      ros::Time last_time_;

      /*!
       * \brief timestamp corresponding to when the command received by the node
       */
      ros::Time cmd_received_timestamp_;

      /*!
       * \brief Input speed command vector represents the desired speed requested by the node. Note that this may differ from the
       * current commanded speed due to acceleration limits imposed by the controller.
       */
      geometry_msgs::Twist cmd_vel_t_;

      /*!
       * \brief speed command vector used internally to represent the current commanded speed
       */
      geometry_msgs::Twist cmd_vel_;

      /*!
       * \brief Input speed command vector represents the desired speed requested by the node.
       */
      geometry_msgs::Twist desired_vel_;

      /*!
       * \brief velocity limits specified externally
       */
      geometry_msgs::Twist max_vel_;

      /*!
       * \brief acceleration limits specified externally
       */
      geometry_msgs::Twist max_accel_;

      /*!
       * \brief maximum translational velocity magnitude allowable
       */
      double max_translational_velocity_;

      /*!
       * \brief maximum rotational velocity magnitude allowable
       */
      double max_rotational_velocity_;

      /*!
       * \brief local gain used for speed control of the caster (to achieve resultant position control)
       */
      double kp_caster_steer_;

      /*!
       * \brief low pass filter value used for finding stalls
       */
      double alpha_stall_;

      /*!
       * \brief vector that stores the wheel controllers
       */
      std::vector<boost::shared_ptr<JointVelocityController> > wheel_controller_;

      /*!
       * \brief vector that stores the caster controllers
       */
      std::vector<boost::shared_ptr<JointVelocityController> > caster_controller_;

      /*!
       * \brief publishes information about the caster and wheel controllers
       */
      boost::scoped_ptr<realtime_tools::RealtimePublisher<pr2_mechanism_controllers::BaseControllerState> > state_publisher_;

      /*!
       * \brief computes the desired caster steers and wheel speeds
       */
      void computeJointCommands(const double &dT);

      /*!
       * \brief tells the wheel and caster controllers to update their speeds
       */
      void updateJointControllers();

      /*!
       * \brief computes the desired caster speeds given the desired base speed
       */
      void computeDesiredCasterSteer(const double &dT);

      /*!
       * \brief computes the desired wheel speeds given the desired base speed
       */
      void computeDesiredWheelSpeeds();

      /*!
       * \brief sends the desired wheel speeds to the wheel controllers
       */
      void setDesiredWheelSpeeds();

      /*!
       * \brief interpolates velocities within a given time based on maximum accelerations
       */
      geometry_msgs::Twist interpolateCommand(const geometry_msgs::Twist &start, const geometry_msgs::Twist &end, const geometry_msgs::Twist &max_rate, const double &dT);

      /*!
       * \brief deal with Twist commands
       */
      void commandCallback(const geometry_msgs::TwistConstPtr& msg);

      /*!
       * \brief callback message, used to remember where the base is commanded to go
       */
      geometry_msgs::Twist base_vel_msg_;

      /*!
       * \brief generic epsilon value that is used as a minimum or maximum allowable input value
       */
      double eps_;

      /*!
       * \brief minimum rotational velocity value allowable
       */
      double cmd_vel_rot_eps_;

      /*!
       * \brief Time interval between state publishing
       */
      double state_publish_time_,state_publish_rate_;

      /*!
       * \brief Time interval between state publishing
       */
      ros::Time last_publish_time_;

      /*!
       * \brief minimum tranlational velocity value allowable
       */
      double cmd_vel_trans_eps_;


      /*!
       * \brief Publish the state
       */
      void publishState(const ros::Time &current_time);

      bool publish_state_;

      /*!
       * \brief The pid controllers for caster position
       */
      std::vector<control_toolbox::Pid> caster_position_pid_;

      filters::MultiChannelTransferFunctionFilter<double> caster_vel_filter_;
      
      std::vector<double> filtered_velocity_;
  };

}
