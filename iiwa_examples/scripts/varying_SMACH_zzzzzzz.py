#!/usr/bin/env python
"""
Varying place position using iiwa robot

prereqursite:

!!Please record the target position offline first!
"""

from iiwa_msgs.msg import JointPosition
from iiwa_examples.srv import *
import sys
import rospy
import copy

import smach
import smach_ros


CurrentJointPos = JointPosition()
pub = None
import ipdb

def hmm_state_switch_client(state):
    rospy.loginfo('wait_for_service #hmm_state_switch#...')
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
                             outcomes=['first', 'second','third','complete'])
        self.counter = 0

    def execute(self, userdata):
        hmm_state_switch_client(0)
        rospy.loginfo('proceeding to the starting position...')

        global CurrentJointPos
        position = [-0.464122414589, 0.671270608902, -0.13456517458, -1.38224005699, 0.0181643459946, 1.11874318123, -2.63843250275]
        tarJPos = CurrentJointPos
        tarJPos.position.a1 =  position[0]
        tarJPos.position.a2 =  position[1]  
        tarJPos.position.a3 =  position[2]
        tarJPos.position.a4 =  position[3]  
        tarJPos.position.a5 =  position[4]
        tarJPos.position.a6 =  position[5]  
        tarJPos.position.a7 =  position[6]
        run(tarJPos)
        rospy.sleep(3)

        if self.counter == 0:
            self.counter += 1
            return 'first'
        elif self.counter == 1:
            self.counter += 1
            return 'second'
        elif self.counter == 2:
            self.counter += 1
            return 'third'
        else:
            self.counter = 0
            return 'complete'

class go_to_first_hover_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 1

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the first hover position...')

        global CurrentJointPos
        position = [-1.71556591988, 0.731243908405, 0.579020619392, -1.6778408289, -0.586030244827, 0.885404765606, -2.93454194069]
        tarJPos = CurrentJointPos
        tarJPos.position.a1 =  position[0]
        tarJPos.position.a2 =  position[1]  
        tarJPos.position.a3 =  position[2]
        tarJPos.position.a4 =  position[3]  
        tarJPos.position.a5 =  position[4]
        tarJPos.position.a6 =  position[5]  
        tarJPos.position.a7 =  position[6]
        run(tarJPos)
        rospy.sleep(3)        
        return 'Succeed'

class go_to_first_picking_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 1

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the first picking position...')

        global CurrentJointPos
        position = [ -1.62472426891,
  1.00216019154,
  0.424003005028,
  -1.63621926308,
  -0.737500667572,
  0.649832129478,
  -2.83222699165]
        tarJPos = CurrentJointPos
        tarJPos.position.a1 =  position[0]
        tarJPos.position.a2 =  position[1]  
        tarJPos.position.a3 =  position[2]
        tarJPos.position.a4 =  position[3]  
        tarJPos.position.a5 =  position[4]
        tarJPos.position.a6 =  position[5]  
        tarJPos.position.a7 =  position[6]
        run(tarJPos)
        rospy.sleep(1)
        return 'Succeed'

class go_to_second_hover_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 2

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the first hover position...')

        global CurrentJointPos
        position = [  -0.963227152824,
   0.863938271999,
   0.538307547569,
   -1.49664449692,
   -0.579189240932,
   0.969517886639,
   -2.28683710098
]
        tarJPos = CurrentJointPos
        tarJPos.position.a1 =  position[0]
        tarJPos.position.a2 =  position[1]  
        tarJPos.position.a3 =  position[2]
        tarJPos.position.a4 =  position[3]  
        tarJPos.position.a5 =  position[4]
        tarJPos.position.a6 =  position[5]  
        tarJPos.position.a7 =  position[6]
        run(tarJPos)
        rospy.sleep(3)    
        return 'Succeed'

class go_to_second_picking_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 2

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the second picking position...')

        global CurrentJointPos
        position = [ -0.911323547363,
  1.10120987892,
  0.429845273495,
  -1.46310663223,
  -0.684240162373,
  0.760607779026,
  -2.22551083565]
        tarJPos = CurrentJointPos
        tarJPos.position.a1 =  position[0]
        tarJPos.position.a2 =  position[1]  
        tarJPos.position.a3 =  position[2]
        tarJPos.position.a4 =  position[3]  
        tarJPos.position.a5 =  position[4]
        tarJPos.position.a6 =  position[5]  
        tarJPos.position.a7 =  position[6]
        run(tarJPos)
        rospy.sleep(1)      
        return 'Succeed'

class go_to_third_hover_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 3

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the first hover position...')

        global CurrentJointPos
        position = [   -0.0665516331792,
   0.921717345715,
  0.610304176807,
  -1.1971899271,
  -0.52401971817,
  1.24567306042,
  -1.50664806366]
        tarJPos = CurrentJointPos
        tarJPos.position.a1 =  position[0]
        tarJPos.position.a2 =  position[1]  
        tarJPos.position.a3 =  position[2]
        tarJPos.position.a4 =  position[3]  
        tarJPos.position.a5 =  position[4]
        tarJPos.position.a6 =  position[5]  
        tarJPos.position.a7 =  position[6]
        run(tarJPos)
        rospy.sleep(3)
        return 'Succeed'

class go_to_third_picking_position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 3

    def execute(self, userdata):
        hmm_state_switch_client(self.state)
        rospy.loginfo('proceeding to the third picking position...')

        global CurrentJointPos
        position = [  -0.0332313552499,
                   1.18277597427,
                   0.507822692394,
                   -1.24154925346,
                  -0.629773139954,
                  0.944730699062,
                      -1.44738340378
]
        tarJPos = CurrentJointPos
        tarJPos.position.a1 =  position[0]
        tarJPos.position.a2 =  position[1]  
        tarJPos.position.a3 =  position[2]
        tarJPos.position.a4 =  position[3]  
        tarJPos.position.a5 =  position[4]
        tarJPos.position.a6 =  position[5]  
        tarJPos.position.a7 =  position[6]
        run(tarJPos)
        rospy.sleep(1)
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
    rospy.init_node("iiwa_joint_position_control", anonymous=True)
    rospy.on_shutdown(shutdown)

    global CurrentJointPos, pub
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size= 10)
    run(None)

    sm = smach.StateMachine(outcomes=['Done'])
    with sm:
        smach.StateMachine.add('go_to_starting_position', go_to_starting_position(),
                               transitions={'first': 'go_to_first_hover_position',
                                           'second': 'go_to_second_hover_position',
                                            'third': 'go_to_third_hover_position',
                                            'complete':'Done'})

        smach.StateMachine.add('go_to_first_hover_position', go_to_first_hover_position(),
                               transitions={'Succeed': 'go_to_first_picking_position'})

        smach.StateMachine.add('go_to_first_picking_position', go_to_first_picking_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

        smach.StateMachine.add('go_to_second_hover_position', go_to_second_hover_position(),
                               transitions={'Succeed': 'go_to_second_picking_position'})

        smach.StateMachine.add('go_to_second_picking_position', go_to_second_picking_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

        smach.StateMachine.add('go_to_third_hover_position', go_to_third_hover_position(),
                               transitions={'Succeed': 'go_to_third_picking_position'})

        smach.StateMachine.add('go_to_third_picking_position', go_to_third_picking_position(),
                               transitions={'Succeed': 'go_to_starting_position'})

    sis = smach_ros.IntrospectionServer('GENETRIC_CLASSIFICATION_VARYING_POSE_SMACH', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
#   rospy.spin()

if __name__ == '__main__':
    sys.exit(main())


