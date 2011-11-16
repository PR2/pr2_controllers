#!/usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
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

import time

import roslib
roslib.load_manifest('robot_mechanism_controllers')
import rospy
from std_msgs.msg import *

rospy.init_node('posture_publisher', disable_signals=True, anonymous=True)

controller = rospy.myargv()[1]
posture = rospy.myargv()[2]


pub_posture = rospy.Publisher("/%s/command_posture" % controller, Float64MultiArray)
postures = {
    'off': [],
    'mantis': [0, 1, 0,  -1, 3.14, -1, 3.14],
    'elbowupr': [-0.79,0,-1.6,  9999, 9999, 9999, 9999],
    'elbowupl': [0.79,0,1.6 , 9999, 9999, 9999, 9999],
    'old_elbowupr': [-0.79,0,-1.6, -0.79,3.14, -0.79,5.49],
    'old_elbowupl': [0.79,0,1.6, -0.79,3.14, -0.79,5.49],
    'elbowdownr': [-0.028262077316910873, 1.2946342642324222, -0.25785640577652386, -1.5498884526859626, -31.278913849571776, -1.0527644894829107, -1.8127318367654268],
    'elbowdownl': [-0.0088195719039858515, 1.2834828245284853, 0.20338442004843196, -1.5565279256852611, -0.096340012666916802, -1.0235018652439782, 1.7990893054129216]
}

while not rospy.is_shutdown():
    m = Float64MultiArray(data = postures[posture])
    pub_posture.publish(m)
    time.sleep(1.0)
