#! /usr/bin/env python

import roslib
roslib.load_manifest('robot_mechanism_controllers')

import rospy, sys
from robot_mechanism_controllers.srv import *
from robot_mechanism_controllers import controllers

def print_usage(exit_code = 0):
    print '''Commands:
    ls                         - List controllers
    set <controller> <command> - Set the controller's commanded value
    setv <controller> <x> <y> <z>  - Set the controller's command as a vector
    get <controller>           - Get the controller's commanded value
    getv <controller>          - Get the controller's vector value 
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print_usage()
    if sys.argv[1] == 'ls':
        controllers.list_controllers()
    elif sys.argv[1] == 'set':
        if len(sys.argv) != 4:
          print_usage()
        controllers.set_controller(sys.argv[2], float(sys.argv[3]))
    elif sys.argv[1] == 'setv':
        if len(sys.argv) < 6:
            print_usage()
        controllers.set_controller_vector(sys.argv[2], map(float, sys.argv[3:6]))
    elif sys.argv[1] == 'get':
        if len(sys.argv) < 3:
          print_usage()
        controllers.get_controller(sys.argv[2])
    elif sys.argv[1] == 'getv':
        if len(sys.argv) < 3:
            print_usage()
        controllers.get_controller_vector(sys.argv[2])
    elif sys.argv[1] == 'setPosition':
        controllers.set_position(sys.argv[2],float(sys.argv[3]))
    elif sys.argv[1] == 'getPosition':
        controllers.get_position(sys.argv[2])
    else:
        print_usage(1)
