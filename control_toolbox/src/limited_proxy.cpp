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

#include <control_toolbox/limited_proxy.h>
#include <algorithm>
#include <cmath>

namespace control_toolbox {

/// REALTIME Utility
//
//   Provide a position error feedback term.  In the linear case this
//   would be Kp*p.  Or, as we are working entirely in acceleration
//   space, it would be lambda^2*p.  Here we extend the feedback term,
//   so that during the entire convergence the expected acceleration
//   will not exceed amax.  This assumes the velocity feedback term is
//   critically damped, i.e. 2*lambda*v.
//
//   lam2	lambda squared (bandwidth of the controller)
//   p          position error
//   amax       max acceleration
//
static double pfbk(double lam2, double p, double amax)
{
  double l2p = lam2*p;

  if      (l2p >  3.0*amax)  return  sqrt(8.0*amax*(+l2p-amax)) - amax;
  else if (l2p > -3.0*amax)  return  l2p;
  else                       return -sqrt(8.0*amax*(-l2p-amax)) + amax;
}

//
//   Provide the matching position gain.  In the linear case this
//   would be just Kp.  Or working with accelerations, lambda^2.  Here
//   we match the cases above to provide the gain in the nonlinear
//   region too.
//
//   lam2	lambda squared (bandwidth of the controller)
//   p          position error
//   amax       max acceleration
//
static double pgain(double lam2, double p, double amax)
{
  double l2p = lam2*p;

  if      (l2p >  3.0*amax)  return lam2 * sqrt(2.0*amax/(+l2p-amax));
  else if (l2p > -3.0*amax)  return lam2;
  else                       return lam2 * sqrt(2.0*amax/(-l2p-amax));
}


void LimitedProxy::update(double des_pos, double des_vel, double des_acc,
                          double pos, double vel, double dt,
                          double &proxy_pos, double &proxy_vel, double &proxy_acc)
{
  double dt2 = dt * dt;

  // The proxy will converge to the desired position, but obeying acceleration limits.
  if (lambda_proxy_ > 0.0)
  {
    // Limits the filter coefficient (lambda) so the bandwidth does not exceed 2/dt.
    double lambda = std::min(lambda_proxy_, 2.0 / dt);
    double lambda2 = lambda * lambda;

    // Simulates the proxy forward by one timestep assuming zero
    // acceleration.  We will later use the difference between the
    // simulated proxy and the desired setpoint to compute the proxy
    // acceleration.
    //
    // The simulation uses a Tustin approximation to integrate the
    // acceleration and velocity.
    double pv = last_proxy_vel_ + 0.5 * dt * (0.0 + last_proxy_acc_);
    double pp = last_proxy_pos_ + 0.5 * dt * ( pv + last_proxy_vel_);

    // Computes the ideal proxy acceleration for tracking the desired setpoint.
    //
    // Limits the acceleration when distant from the setpoint to avoid
    // building up velocity.  If the system is moving too quickly, it
    // will be unable to decelerate and will overshoot the setpoint.
    proxy_acc = (des_acc - 2.0*lambda*(pv - des_vel) - pfbk(lambda2, pp - des_pos, acc_limit_)) /
      (1.0 + 2.0*lambda*0.5*dt + pgain(lambda2, pp - des_pos, acc_limit_)*0.25*dt2);

    if (vel_limit_ > 0.0) {
      double acc_hi = -lambda * (pv - vel_limit_) / (1.0 + lambda * 0.5 * dt);
      double acc_lo = -lambda * (pv + vel_limit_) / (1.0 + lambda * 0.5 * dt);
      proxy_acc = std::max(acc_lo, std::min(proxy_acc, acc_hi));
    }

    // Enforces the acceleration limits of the proxy.
    /*
    if (acc_limit_ > 0.0) {
      proxy_acc = std::max(-acc_limit_, std::min(proxy_acc, acc_limit_));
    }
    */

    // Simulates the proxy forward one timestep using the computed acceleration.
    proxy_vel = last_proxy_vel_ + 0.5 * dt * (proxy_acc + last_proxy_acc_);
    proxy_pos = last_proxy_pos_ + 0.5 * dt * (proxy_vel + last_proxy_vel_);
  }
  else
  {
    // The proxy is turned off, so just set it to track the desired exactly.
    proxy_acc = des_acc;
    proxy_vel = des_vel;
    proxy_pos = des_pos;
  }

  // Calculates the effort needed to move the mechanism to the proxy.
  double effort = mass_ * proxy_acc - Kd_*(vel - proxy_vel) - Kp_*(pos - proxy_pos);

  // Adjusts the proxy so that the force does not saturate.
  if (effort_limit_ > 0.0 && abs(effort) > effort_limit_)
  {
    double limited_effort = std::max(-effort_limit_, std::min(effort, effort_limit_));

    // Updates the proxy based on which gains are effective.
    if (mass_ > 0.0) {
      double delta_acc = (limited_effort - effort) / (mass_ + 0.5*dt*Kd_ + 0.25*dt2*Kp_);

      proxy_acc += delta_acc;
      proxy_vel += delta_acc * 0.5*dt;
      proxy_pos += delta_acc * 0.25*dt2;
    }
    else if (Kd_ > 0.0) {
      double delta_vel = (limited_effort - effort) / (Kd_ + 0.5*dt*Kp_);

      proxy_vel += delta_vel;
      proxy_pos += delta_vel * 0.5*dt;
    }
    else if (Kp_ > 0.0) {
      double delta_pos = (limited_effort - effort) / Kp_;

      proxy_pos += delta_pos;
    }
  }

  // Remembers the last results
  last_proxy_pos_ = proxy_pos;
  last_proxy_vel_ = proxy_vel;
  last_proxy_acc_ = proxy_acc;
}

} // namespace
