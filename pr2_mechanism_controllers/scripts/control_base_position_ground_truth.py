#!/usr/bin/env python
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

# Author: Vijay Pradeep
# (Derived from pointhead.py)

PKG = "pr2_mechanism_controllers"

import roslib; roslib.load_manifest(PKG)

import sys
import os
import string
from time import sleep

import rospy
from geometry_msgs.msg import PointStamped, Point


def control_base_pose_odom_frame(x,y,w):
    head_angles = rospy.Publisher('ground_truth_controller/set_cmd', Point)
    rospy.init_node('base_position_commander', anonymous=True)
    p = Point(x, y, w) ;
    sleep(1)
    head_angles.publish( p  )
    sleep(1)

def usage():
    return "%s [pos_x] [pos_y] [theta (rad)]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) ==4:
        control_base_pose_odom_frame(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]) )
    else:
        print usage()
        sys.exit(1)

