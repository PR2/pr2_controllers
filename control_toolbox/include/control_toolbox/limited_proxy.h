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
#ifndef CONTROL_TOOLBOX_LIMITED_PROXY_H
#define CONTROL_TOOLBOX_LIMITED_PROXY_H

namespace control_toolbox {

class LimitedProxy
{
public:

  double lambda_proxy_;
  double acc_limit_;  /// Acceleration limit of the proxy
  double vel_limit_;  /// Velocity limit of the proxy
  double effort_limit_;  /// Effort limit of the mechanism
  double mass_;  /// Estimated mass of the joint
  double Kp_, Kd_; /// Gains used to control the actual to track the proxy

  LimitedProxy()
    : lambda_proxy_(0.0), acc_limit_(0.0), effort_limit_(0.0),
      mass_(0.0)
  {
  }

  
  void setState(double proxy_pos, double proxy_vel, double proxy_acc) {
    last_proxy_pos_ = proxy_pos;
    last_proxy_vel_ = proxy_vel;
    last_proxy_acc_ = proxy_acc;
  }
  
  void getState(double &proxy_pos, double &proxy_vel, double &proxy_acc) {
    proxy_pos = last_proxy_pos_;
    proxy_vel = last_proxy_vel_;
    proxy_acc = last_proxy_acc_;
  }

  void update(double des_pos, double des_vel, double des_acc,
              double pos, double vel, double dt,
              double &proxy_pos, double &proxy_vel, double &proxy_acc);

private:
  double last_proxy_pos_, last_proxy_vel_, last_proxy_acc_;
};
  
} // namespace

#endif
