#!/usr/bin/env python
"""
Randomly varying place position using iiwa robot

prereqursite:

!!Please record the target position offline first!
"""

from geometry_msgs.msg import PoseStamped 
from iiwa_examples.srv import *
import sys
import rospy
import copy
import numpy as np

import smach
import smach_ros


CurrentPose = PoseStamped()
nPose = 10
# the starting pose
POSE_X = 0.5
POSE_Y = -0.3
POSE_Z = 0.3
POSE_oX = 0.0
POSE_oY = 1.0
POSE_oZ = 0.0
POSE_oW = 0.0
pub = None
import ipdb

def hmm_state_switch_client(state):
    rospy.loginfo('wait_for_service #hmm_state_switch# from by real_tag_multimodal...')
    rospy.wait_for_service('hmm_state_switch')
    try:
        hmm_state_switch_proxy = rospy.ServiceProxy('hmm_state_switch', State_Switch)
        req = State_SwitchRequest()
        req.state = state
        resp = hmm_state_switch_proxy(req)
        if resp.finish.data:
            print "Hmm State switch to %d succesfully" %state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


class go_to_starting_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['randoming',  'complete'])
        self.counter = 0

    def execute(self, userdata):
        hmm_state_switch_client(0)
        rospy.loginfo('proceeding to the starting position...')
        position = [POSE_X, POSE_Y, POSE_Z + 0.2, POSE_oX, POSE_oY, POSE_oZ, POSE_oW]
        print position
        cartPose = CurrentPose
        cartPose.pose.position.x =  position[0]
        cartPose.pose.position.y =  position[1]  
        cartPose.pose.position.z =  position[2]
        cartPose.pose.orientation.x =  position[3]  
        cartPose.pose.orientation.y =  position[4]
        cartPose.pose.orientation.z =  position[5]  
        cartPose.pose.orientation.w =  position[6]
        run(cartPose)
        rospy.sleep(2)        

        while self.counter < nPose
            self.counter += 1
            return 'randoming'
        return 'complete'

class go_to_random_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 1

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the randoming position...')
        position = [POSE_X     + np.random.uniform(-0.1,0.1),
                    POSE_Y-0.5 + np.random.uniform(-0.1, 0.1), 
                    POSE_Z, 
                    POSE_oX, 
                    POSE_oY, 
                    POSE_oZ, 
                    POSE_oW]
        cartPose = CurrentPose
        cartPose.pose.position.x =  position[0]
        cartPose.pose.position.y =  position[1]  
        cartPose.pose.position.z =  position[2]
        cartPose.pose.orientation.x =  position[3]  
        cartPose.pose.orientation.y =  position[4]
        cartPose.pose.orientation.z =  position[5]  
        cartPose.pose.orientation.w =  position[6]
        run(cartPose)
        rospy.sleep(3)        
        return 'Succeed'

def run(cartPose):
    global pub
    if cartPose is not None:
        try:
            pub.publish(cartPose)
            rospy.loginfo(cartPose)
        except:
            rospy.loginfo('the target is missing, so no publish')
    else:
        rospy.loginfo('waiting for msg')

def shutdown():
    rospy.loginfo("Stopping the node...")


def main():
    rospy.init_node("iiwa_cartesian_pose_control", anonymous=True)
    rospy.on_shutdown(shutdown)

    global pub
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size= 10)
    run(None)

    sm = smach.StateMachine(outcomes=['Done'])
    with sm:
        smach.StateMachine.add('go_to_starting_position', go_to_starting_position(),
                               transitions={
                                            'randoming': 'go_to_random_position',
                                            'complete' : 'Done'})

        smach.StateMachine.add('go_to_random_position', go_to_random_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

    sis = smach_ros.IntrospectionServer('GENETRIC_CLASSIFICATION_VARYING_POSE_SMACH', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    sys.exit(main())


