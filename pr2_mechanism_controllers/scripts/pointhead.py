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

# Author: Melonee Wise

PKG = "pr2_mechanism_controllers"

import roslib; roslib.load_manifest(PKG)

import sys
import os
import string
from time import sleep

import rospy
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import JointState

def point_head_client(pan, tilt):

    head_angles = rospy.Publisher('head_controller/command', JointState)
    rospy.init_node('head_commander', anonymous=True)
    sleep(1)
    js = JointState()
    js.name = ['head_pan_joint', 'head_tilt_joint'];
    js.position = [pan,tilt];
    head_angles.publish(js)
    sleep(1)

def point_head_cart_client(x,y,z,frame):

    head_angles = rospy.Publisher('head_controller/point_head', PointStamped)
    rospy.init_node('head_commander', anonymous=True)
    sleep(1)
    head_angles.publish(PointStamped(rospy.Header(None, rospy.get_rostime(), frame), Point(x, y, z)))
    sleep(1)


def usage():
    return "%s [pan tilt] or [x,y,z,frame]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) < 3:
        print usage()
        sys.exit(1)
    elif len(sys.argv) ==3:
        point_head_client(float(sys.argv[1]), float(sys.argv[2]))
    elif len(sys.argv) ==5:
        point_head_cart_client(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4])
