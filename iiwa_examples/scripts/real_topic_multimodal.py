#!/usr/bin/env python
import argparse
import struct
import sys
import copy
import ipdb
import ipdb
import rospy
from std_msgs.msg import (
    Empty,
    Header
)

import copy

from iiwa_msgs.msg import CartesianVelocity
from iiwa_msgs.msg import CartesianEulerPose
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointVelocity
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped

from iiwa_examples.msg import Tag_MultiModal
from iiwa_examples.srv import (
    State_Switch,
    State_SwitchResponse
)

shared_header = Header()
shared_endpoint_state = PoseStamped() 
def callback_Pose(endpoint_state):
    global shared_header
    global shared_endpoint_state 
    shared_header = endpoint_state.header
    shared_endpoint_state = endpoint_state

shared_joint_state = JointPosition()
def callback_jPos(joint_state):
    global shared_joint_state
    shared_joint_state = joint_state

shared_wrench_stamped = WrenchStamped() 
def callback_wrench_stamped(wrench_stamped):
    global shared_wrench_stamped
    shared_wrench_stamped = wrench_stamped

hmm_state = None 
def state_switch_handle(req):
    global hmm_state
    hmm_state = req.state
    print "state is changed to %d" %req.state
    resp = State_SwitchResponse()
    resp.finish.data = True
    return resp
    

def main():
    global hmm_state
    global shared_header
    global shared_endpoint_state 
    global shared_joint_state
    global shared_wrench_stamped

    hmm_state = 0

    publishing_rate = 100
    
    rospy.init_node("topic_multimodal", anonymous=True)
    rospy.Subscriber("/iiwa/state/CartesianPose", PoseStamped, callback_Pose)
    rospy.Subscriber("/iiwa/state/JointPosition", JointPosition, callback_jPos)
    rospy.Subscriber("/iiwa/state/CartesianWrench", WrenchStamped, callback_wrench_stamped)

    pub = rospy.Publisher("/tag_multimodal",Tag_MultiModal, queue_size=10)

    state_switch = rospy.Service('hmm_state_switch', State_Switch, state_switch_handle)

    r = rospy.Rate(publishing_rate)
    
    while not rospy.is_shutdown():
        tag_multimodal = Tag_MultiModal()
        tag_multimodal.tag = hmm_state
        tag_multimodal.header = copy.deepcopy(shared_header)
        tag_multimodal.CartesianPose = copy.deepcopy(shared_endpoint_state)
        tag_multimodal.JointPosition = copy.deepcopy(shared_joint_state)
        tag_multimodal.CartesianWrench = copy.deepcopy(shared_wrench_stamped)
        pub.publish(tag_multimodal)

        r.sleep()

if __name__ == '__main__':
    sys.exit(main())
