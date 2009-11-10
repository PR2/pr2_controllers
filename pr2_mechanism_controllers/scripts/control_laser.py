#!/usr/bin/env python

PKG = "pr2_mechanism_controllers"

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string

import rospy
from std_msgs import *

from pr2_mechanism_controllers.srv import *

def print_usage(exit_code = 0):
    print '''Usage:
    control.py <controller> sine <period> <amplitude> <offset>
       - Defines a sine sweep for the laser controller
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) < 6:
        print_usage()
    if sys.argv[2] == 'sine' :
        profile_type = 4            # Implies a sine sweep
        controller = sys.argv[1]
        period =     float (sys.argv[3])
        amplitude =  float (sys.argv[4])
        offset =     float (sys.argv[5])

        print 'Sending Command: '
        print '  Profile Type: Sine'
        print '  Period:       %f Seconds' % period
        print '  Amplitude:    %f Radians' % amplitude
        print '  Offset:       %f Radians' % offset

        rospy.wait_for_service(controller + '/set_profile')
        s = rospy.ServiceProxy(controller + '/set_profile', SetProfile)
        resp = s.call(SetProfileRequest(0.0, 0.0, 0.0, 0.0, profile_type, period, amplitude, offset))
        
        print 'Command Sent'
        print '  Response: %s' % str(resp.time)
    else :
        print 'Unknown profile type [%s]' % sys.argv[2]

