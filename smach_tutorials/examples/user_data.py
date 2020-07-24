#!/usr/bin/env python
"""
Description:
    Create a two-state state machine where one state writes to userdata and
    the other state reads from userdata, and spews a message to rosout.

Usage:
    $> ./user_data.py

Output:
    [INFO] : State machine starting in initial state 'SET' with userdata: 
        []
    [INFO] : State machine transitioning 'SET':'set_it'-->'GET'
    [INFO] : >>> GOT DATA! x = True
    [INFO] : State machine terminating 'GET':'got_it':'succeeded'
"""

import rospy
import smach
import smach_ros

class Setter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['set_it'], output_keys = ['x'])
    def execute(self, ud):
        ud.x = True
        return 'set_it'

class Getter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['got_it'], input_keys = ['x'])
    def execute(self, ud):
        rospy.loginfo('>>> GOT DATA! x = '+str(ud.x))
        return 'got_it'

def main():
    rospy.init_node('smach_example_user_data')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SET', Setter(), {'set_it':'GET'})
        smach.StateMachine.add('GET', Getter(), {'got_it':'succeeded'})

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
