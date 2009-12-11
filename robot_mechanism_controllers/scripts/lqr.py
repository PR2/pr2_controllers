# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('robot_mechanism_controllers')

from robot_mechanism_controllers.lqr_controller import *
foo=LQRProxy('right_arm_controller')
foo.cmd(['r_shoulder_pan_joint',\
      'r_shoulder_lift_joint',\
      "r_upper_arm_roll_joint",\
      "r_upper_arm_joint",\
      "r_elbow_flex_joint",\
      "r_forearm_roll_joint",\
      "r_wrist_flex_joint"],\
    [-1,0.5,-1.0,1.0,-1,1,1],\
    #[-1,-0.2,-1,-1,-1,-1,-1],\
    [0.0,0,0,0,0,0,0],\
    [1000000,10000,1000,1000,1000,1000,1000,100,00,10,10,10,10,1],\
    #[10,1,1,1,1,  0,0,0,0,0],\
    [10,10,10,10,10,100,100])
