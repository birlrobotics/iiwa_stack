#!/usr/bin/env python

import sys
import rospy
from iiwa_msgs.srv import *

POSITION_CONTROL = 0
JOINT_IMPEDANCE  = 1
CARTESIAN_IMPEDANCE = 2
DESIRED_FORCE = 3
SINE_PATTERN = 4

config = ConfigureSmartServo() 
def main():
    rospy.wait_for_service("/iiwa/configuration/configureSmartServo")
    try:
        mode_config_proxy = rospy.ServiceProxy('/iiwa/configuration/configureSmartServo', ConfigureSmartServo)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    config = ConfigureSmartServoRequest()
    config.control_mode = CARTESIAN_IMPEDANCE
    config.cartesian_impedance.cartesian_stiffness.x = 0.0
    config.cartesian_impedance.cartesian_stiffness.y = 0.0
    config.cartesian_impedance.cartesian_stiffness.z = 0.0
    config.cartesian_impedance.cartesian_stiffness.a = 1000.0
    config.cartesian_impedance.cartesian_stiffness.b = 1000.0
    config.cartesian_impedance.cartesian_stiffness.c = 1000.0
    config.cartesian_impedance.cartesian_damping.x = 1
    config.cartesian_impedance.cartesian_damping.y = 1
    config.cartesian_impedance.cartesian_damping.z = 1
    config.cartesian_impedance.cartesian_damping.a = 1
    config.cartesian_impedance.cartesian_damping.b = 1
    config.cartesian_impedance.cartesian_damping.c = 1
    config.cartesian_impedance.nullspace_stiffness = 0.6
    config.cartesian_impedance.nullspace_damping = 0.7
    respo = mode_config_proxy(config)
    print ("Current_control_model:%.d" % config.control_mode)    
if __name__=="__main__":
    sys.exit(main())
