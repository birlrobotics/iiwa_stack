#!/usr/bin/env python
"""
Varying place position using iiwa robot

prereqursite:

!!Please record the target position offline first!
"""

from geometry_msgs.msg import PoseStamped 
from iiwa_examples.srv import *
import sys
import rospy
import copy

import smach
import smach_ros


CurrentPose = PoseStamped()
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
                             outcomes=['first', 'second','third', 'fourth', 'fifth', 'sixth',  'complete'])
        self.counter = 1

    def execute(self, userdata):
        hmm_state_switch_client(0)
        rospy.loginfo('proceeding to the starting position...')
        position = [POSE_X, POSE_Y, POSE_Z + 0.2, POSE_oX, POSE_oY, POSE_oZ, POSE_oW]
        print position
        tarJPos = CurrentPose
        tarJPos.pose.position.x =  position[0]
        tarJPos.pose.position.y =  position[1]  
        tarJPos.pose.position.z =  position[2]
        tarJPos.pose.orientation.x =  position[3]  
        tarJPos.pose.orientation.y =  position[4]
        tarJPos.pose.orientation.z =  position[5]  
        tarJPos.pose.orientation.w =  position[6]
        run(tarJPos)
        rospy.sleep(2)        

        if self.counter == 1:
            self.counter += 1
            return 'first'
        elif self.counter == 2:
            self.counter += 1
            return 'second'
        elif self.counter == 3:
            self.counter += 1
            return 'third'
        elif self.counter == 4:
            self.counter += 1
            return 'fourth'
        elif self.counter == 5:
            self.counter += 1
            return 'fifth'
        elif self.counter == 6:
            self.counter += 1
            return 'sixth'
        else:
            self.counter = 0
            return 'complete'

class go_to_first_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 1

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the first hover position...')
        position = [POSE_X, POSE_Y-0.5, POSE_Z, POSE_oX, POSE_oY, POSE_oZ, POSE_oW]
        tarJPos = CurrentPose
        tarJPos.pose.position.x =  position[0]
        tarJPos.pose.position.y =  position[1]  
        tarJPos.pose.position.z =  position[2]
        tarJPos.pose.orientation.x =  position[3]  
        tarJPos.pose.orientation.y =  position[4]
        tarJPos.pose.orientation.z =  position[5]  
        tarJPos.pose.orientation.w =  position[6]
        run(tarJPos)
        rospy.sleep(3)        
        return 'Succeed'


class go_to_second_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 2

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the first picking position...')
        position = [POSE_X, POSE_Y-0.3, POSE_Z, POSE_oX, POSE_oY, POSE_oZ, POSE_oW]
        tarJPos = CurrentPose
        tarJPos.pose.position.x =  position[0]
        tarJPos.pose.position.y =  position[1]  
        tarJPos.pose.position.z =  position[2]
        tarJPos.pose.orientation.x =  position[3]  
        tarJPos.pose.orientation.y =  position[4]
        tarJPos.pose.orientation.z =  position[5]  
        tarJPos.pose.orientation.w =  position[6]
        run(tarJPos)
        rospy.sleep(3)
        return 'Succeed'

class go_to_third_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 3

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the first hover position...')
        position = [POSE_X, POSE_Y-0.1, POSE_Z, POSE_oX, POSE_oY, POSE_oZ, POSE_oW]
        tarJPos = CurrentPose
        tarJPos.pose.position.x =  position[0]
        tarJPos.pose.position.y =  position[1]  
        tarJPos.pose.position.z =  position[2]
        tarJPos.pose.orientation.x =  position[3]  
        tarJPos.pose.orientation.y =  position[4]
        tarJPos.pose.orientation.z =  position[5]  
        tarJPos.pose.orientation.w =  position[6]
        run(tarJPos)
        rospy.sleep(2)    
        return 'Succeed'

class go_to_fourth_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 4

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the second picking position...')
        position = [POSE_X, POSE_Y+0.1, POSE_Z, POSE_oX, POSE_oY, POSE_oZ, POSE_oW]
        tarJPos = CurrentPose
        tarJPos.pose.position.x =  position[0]
        tarJPos.pose.position.y =  position[1]  
        tarJPos.pose.position.z =  position[2]
        tarJPos.pose.orientation.x =  position[3]  
        tarJPos.pose.orientation.y =  position[4]
        tarJPos.pose.orientation.z =  position[5]  
        tarJPos.pose.orientation.w =  position[6]
        run(tarJPos)
        rospy.sleep(2)      
        return 'Succeed'

class go_to_fifth_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 5

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the first hover position...')
        position = [POSE_X, POSE_Y+0.3, POSE_Z, POSE_oX, POSE_oY, POSE_oZ, POSE_oW]
        tarJPos = CurrentPose
        tarJPos.pose.position.x =  position[0]
        tarJPos.pose.position.y =  position[1]  
        tarJPos.pose.position.z =  position[2]
        tarJPos.pose.orientation.x =  position[3]  
        tarJPos.pose.orientation.y =  position[4]
        tarJPos.pose.orientation.z =  position[5]  
        tarJPos.pose.orientation.w =  position[6]
        run(tarJPos)
        rospy.sleep(2)
        return 'Succeed'

class go_to_sixth_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 6

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the third picking position...')
        position = [POSE_X, POSE_Y+0.5, POSE_Z, POSE_oX, POSE_oY, POSE_oZ, POSE_oW]
        tarJPos = CurrentPose
        tarJPos.pose.position.x =  position[0]
        tarJPos.pose.position.y =  position[1]  
        tarJPos.pose.position.z =  position[2]
        tarJPos.pose.orientation.x =  position[3]  
        tarJPos.pose.orientation.y =  position[4]
        tarJPos.pose.orientation.z =  position[5]  
        tarJPos.pose.orientation.w =  position[6]
        run(tarJPos)
        rospy.sleep(3)
        return 'Succeed'

def run(tarJPos):
    global pub
    if tarJPos is not None:
        try:
            pub.publish(tarJPos)
            rospy.loginfo(tarJPos)
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
                               transitions={'first':  'go_to_first_position',
                                            'second':  'go_to_second_position',
                                            'third':  'go_to_third_position',
                                            'fourth': 'go_to_fourth_position',
                                            'fifth':  'go_to_fifth_position',
                                            'sixth':  'go_to_sixth_position',
                                            'complete':'Done'})

        smach.StateMachine.add('go_to_first_position', go_to_first_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

        smach.StateMachine.add('go_to_second_position', go_to_second_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

        smach.StateMachine.add('go_to_third_position', go_to_third_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

        smach.StateMachine.add('go_to_fourth_position', go_to_fourth_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

        smach.StateMachine.add('go_to_fifth_position', go_to_fifth_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

        smach.StateMachine.add('go_to_sixth_position', go_to_sixth_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

    sis = smach_ros.IntrospectionServer('GENETRIC_CLASSIFICATION_VARYING_POSE_SMACH', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
#   rospy.spin()

if __name__ == '__main__':
    sys.exit(main())


