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
#pragma once

namespace control_toolbox {
/***************************************************/
/*! \class Ramp
    \brief A basic ramp class.

    This class basically calculates the output for
    a ramp. 

*/
/***************************************************/

class Ramp
{
public:

  /*!
   * \brief Constructor
   */
  Ramp();

  /*!
   * \brief Destructor.
   */
  ~Ramp();

  /*!
   * \brief Update the SineSweep loop with nonuniform time step size.
   *
   * \param dt Change in time since last call
   */
  double update(double dt);

  /*!
   * \brief Intializes everything and calculates the constants for the sweep.
   *
   * \param output_start  Start command of the ramp.
   * \param output_end  End command of the ramp.
   * \param duration  The duration of the ramp.
   */
  void init(double output_start, double output_end, double duration);

private:
  double output_start_;           /**< Begining of the ramp. */
  double output_end_;             /**< End of the ramp. */
  double duration_;               /**< Duration of the ramp. */
  double cmd_;                    /**< Command to send. */
};
}
