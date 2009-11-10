#!/usr/bin/env python

PKG = "pr2_mechanism_controllers"

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string

import rospy
from std_msgs import *

from geometry_msgs.msg import Point
from pr2_mechanism_controllers.msg import TrackLinkCmd
from time import sleep

def print_usage(exit_code = 0):
    print '''Usage:
    send_track_link_cmd.py [controller] [link_name] [x] [y] [z]
       - [controller] - The controller's name
       - [link name]  - Name of the link that we want to track
       - [x] [y] [z]  - The location on the link (in the link's frame) that we want to track
    send_track_link_cmd.py [controller]
       - Sends a 'disable' command to the controller
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) == 6 :
        cmd = TrackLinkCmd()
        controller =    sys.argv[1]
        cmd.link_name = sys.argv[2] 
        cmd.point     = Point(float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]))
        cmd.enable    = 1
    elif len(sys.argv) == 2 :
        cmd = TrackLinkCmd()
        controller =    sys.argv[1]
        cmd.link_name = 'none'
        cmd.point     = Point(0.0, 0.0, 0.0)
        cmd.enable    = 0
    else :
        print_usage()

    print 'Sending TrackLinkCmd Command to %s: ' % controller
    print '  Enable:    %u' % cmd.enable
    print '  Link Name: %s' % cmd.link_name
    print '  Point:     (%f, %f, %f)' % (cmd.point.x, cmd.point.y, cmd.point.z)

    command_publisher = rospy.Publisher(controller + '/set_track_link_cmd', TrackLinkCmd)
    rospy.init_node('track_link_cmd_commander', anonymous=True)
    sleep(1)
    command_publisher.publish( cmd )

    sleep(1)
    print 'Command sent!'

