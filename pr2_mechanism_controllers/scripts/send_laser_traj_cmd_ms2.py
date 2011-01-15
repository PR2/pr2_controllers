#!/usr/bin/env python

PKG = "pr2_mechanism_controllers"

import roslib; roslib.load_manifest(PKG)

import sys
import os
import string

import rospy
from std_msgs import *

from pr2_msgs.msg import LaserTrajCmd
from pr2_msgs.srv import *
from time import sleep

def print_usage(exit_code = 0):
    print '''Usage:
    send_periodic_cmd.py [controller]
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage()

    cmd = LaserTrajCmd()
    controller   =    sys.argv[1]
    cmd.header   =    rospy.Header(None, None, None)
    cmd.profile  = "blended_linear"
    #cmd.pos      = [1.0, .26, -.26, -.7,   -.7,   -.26,   .26,   1.0, 1.0]
    d = .025
    #cmd.time     = [0.0, 0.4,  1.0, 1.1, 1.1+d,  1.2+d, 1.8+d, 2.2+d, 2.2+2*d]

    cmd.position        = [1.2,  -.7, 1.2]
    cmd.time_from_start = [0.0, 1.8, 2.025]
    cmd.time_from_start = [rospy.Duration.from_sec(x) for x in cmd.time_from_start]

    cmd.max_velocity     =  5
    cmd.max_acceleration =  5

    print 'Sending Command to %s: ' % controller
    print '  Profile Type: %s' % cmd.profile
    print '  Pos: %s ' % ','.join(['%.3f' % x for x in cmd.position])
    print '  Time: %s' % ','.join(['%.3f' % x.to_sec() for x in cmd.time_from_start])
    print '  MaxVelocity:     %f' % cmd.max_velocity
    print '  MaxAcceleration: %f' % cmd.max_acceleration

    rospy.wait_for_service(controller + '/set_traj_cmd')
    s = rospy.ServiceProxy(controller + '/set_traj_cmd', SetLaserTrajCmd)
    resp = s.call(SetLaserTrajCmdRequest(cmd))

    print 'Command sent!'
    print '  Resposne: %f' % resp.start_time.to_seconds()
