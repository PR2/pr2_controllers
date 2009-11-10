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

#include <pr2_mechanism_controllers/Pose3D.h>
#include <ros/node.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <fstream>

static int done = 0;

void finalize(int donecare)
{
  done = 1;
}


////////////////////////////////////////////////////////////////////////////////
// Return the rotation in Euler angles
libTF::Vector GetAsEuler(geometry_msgs::Quaternion quat)
{
  libTF::Vector vec;

  double squ;
  double sqx;
  double sqy;
  double sqz;

//  this->Normalize();

  squ = quat.w * quat.w;
  sqx = quat.x * quat.x;
  sqy = quat.y * quat.y;
  sqz = quat.z * quat.z;

  // Roll
  vec.x = atan2(2 * (quat.y*quat.z + quat.w*quat.x), squ - sqx - sqy + sqz);

  // Pitch
  vec.y = asin(-2 * (quat.x*quat.z - quat.w * quat.y));

  // Yaw
  vec.z = atan2(2 * (quat.x*quat.y + quat.w*quat.z), squ + sqx - sqy - sqz);

  return vec;
}


class test_run_base
{
  public:

    test_run_base(){}; 

    ~test_run_base() {}

    nav_msgs::Odometry odom;

    void odomMsgReceived()
    {
    };

};

int main( int argc, char** argv )
{

  /*********** Initialize ROS  ****************/
  ros::init(argc,argv);
  ros::Node *node = new ros::Node("test_run_base_controller"); 

  test_run_base tb;

  node->subscribe("odom",tb.odom,&test_run_base::odomMsgReceived,&tb,10);

  signal(SIGINT,  finalize);
  signal(SIGQUIT, finalize);
  signal(SIGTERM, finalize);


  /*********** Start moving the robot ************/
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;

  double run_time = 0;
  bool run_time_set = false;
  int file_num = 0;

  if(argc >= 2)
    cmd.linear.x = atof(argv[1]);

  if(argc >= 3)
    cmd.linear.y = atof(argv[2]);

  if(argc >= 4)
    cmd.angular.z = atof(argv[3]);

  if(argc >=5)
  { 
     run_time = atof(argv[4]);
     run_time_set = true;
  }

  if(argc ==6)
  {
     file_num = atoi(argv[5]);
  }

  node->advertise<geometry_msgs::Twist>("cmd_vel",10);
  sleep(1);

  libTF::Vector ang_rates;
  ros::Time start_time = ros::Time::now();
  ros::Duration sleep_time = ros::Duration().fromSec(0.01);

  std::ofstream odom_log_file;
  odom_log_file.open("odom_log.txt");


  while(!done)
  {
     ros::Duration delta_time = ros::Time::now() - start_time;

     if(run_time_set && delta_time.toSec() > run_time)
        break;

     odom_log_file << tb.odom.pose.pose.position.x <<  " " << tb.odom.pose.pose.position.y << " " << tf::getYaw(tb.odom.pose.pose.orientation) << " " << tb.odom.twist.twist.linear.x <<  " " << tb.odom.twist.twist.linear.y << " " << tb.odom.twist.twist.angulat.z << " " << tb.odom.header.stamp.sec + tb.odom.header.stamp.nsec/1.0e9 << std::endl;

     cout << endl << "odometry:: " << endl << "velocity:" << endl << " x:" << tb.odom.twist.twist.linear.x <<  endl << " y:" << tb.odom.twist.twist.linear.y << endl << " omega:" << tb.odom.twist.twist.angular.z << std::endl;
    node->publish("cmd_vel",cmd);
    sleep_time.sleep();
  }

  node->unsubscribe("odom");
  node->unadvertise("cmd_vel");

  odom_log_file.close();

  
}
