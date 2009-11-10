#! /usr/bin/python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import roslib; roslib.load_manifest('pr2_mechanism_controllers')
roslib.load_manifest('rospy')

import random, time
import rospy
import sys
from std_msgs.msg import *
from manipulation_msgs.msg import TaskFrameFormalism

pub = rospy.Publisher('r_arm_cartesian_tff_controller/command', TaskFrameFormalism)

def p(mx, vx, my, vy, mz, vz, mxx, vxx, myy, vyy, mzz, vzz):
  m = TaskFrameFormalism()
  m.header.frame_id = ''
  m.header.stamp = rospy.get_rostime()
  m.mode.linear.x = mx
  m.mode.linear.y = my
  m.mode.linear.z = mz
  m.mode.angular.x = mxx
  m.mode.angular.y = myy
  m.mode.angular.z = mzz
  m.value.linear.x = vx
  m.value.linear.y = vy
  m.value.linear.z = vz
  m.value.angular.x = vxx
  m.value.angular.y = vyy
  m.value.angular.z = vzz
  pub.publish(m)



def main():
    rospy.init_node('pub', anonymous=True)
    time.sleep(2)
    if len(sys.argv) < 12:
        print "Usage:  tff.py mx, vx, my, vy, mz, vz, mxx, vxx, myy, vyy, mzz, vzz"
        sys.exit(1)
    p(float(sys.argv[1]), float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6]),float(sys.argv[7]),float(sys.argv[8]),float(sys.argv[9]),float(sys.argv[10]),float(sys.argv[11]),float(sys.argv[12]))


if __name__ == '__main__':
    main()
