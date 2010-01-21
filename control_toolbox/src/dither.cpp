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

// Original version: Kevin Watts <watts@willowgarage.com>

#include <control_toolbox/dither.h>

// Constants to generate white noise
#define M 714025
#define IA 1366
#define IC 150889
#define LM 2147483647
#define LAM (1.0/LM)
#define LA 16807
#define LR 2836
#define LQ 127773


namespace control_toolbox {

Dither::Dither() : amplitude_(0), s_(0), x_(0), idum((long)0)
{

}

Dither::~Dither()
{
}

double Dither::update()
{
  // Shamelessly copied from 
  // http://www.koders.com/cpp/fidB74E9B684392743A5D3DE72227863382884AB0A2.aspx?s=Chebyshev
  
  double fac, r, v1, v2;
  
  if (s_ == 0)
  {
    for (int i = 0; i < 100; i++)
    {
      v1 = (2.0 * uni()) - 1.0;
      v2 = (2.0 * uni()) - 1.0;
      r = (v1 * v1) + (v2 * v2);
      
      if (r < 1.0)
        break;
    }
    if (r >= 1.0)
      r = 1.0;
    
    fac = sqrt(-2.0 * log(r) / r);
    x_ = v1 * fac;
    s_ = 1;
    return v2 * fac * amplitude_;
  }
  else
  {
    s_ = 0;
    return x_ * amplitude_;
  }
}


//--------------------------------------------------------------------
//       Returns uniform deviate between 0.0 and 1.0.
//       Used to generate PN data
//---------------------------------------------------------------------
double Dither::uni()
{
  double rm, r1;
  rm  = 1./M;
  idum = (long)fmod(IA * idum + IC, M);
  r1 = idum * rm;
  return(r1);
}


}
