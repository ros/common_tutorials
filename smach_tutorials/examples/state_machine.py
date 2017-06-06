#!/usr/bin/env python
"""
Description:
    Create a simple 3-state state machine.

Usage:
    $> ./state_machine.py

Output:
    [INFO] : State machine starting in initial state 'FOO' with userdata: 
            []
    [INFO] : State machine transitioning 'FOO':'done'-->'BAR'
    [INFO] : State machine transitioning 'BAR':'done'-->'BAZ'
    [INFO] : State machine terminating 'BAZ':'done':'succeeded'

"""

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

class ExampleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['done'])
    def execute(self, ud):
        return 'done'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', ExampleState(), {'done':'BAR'})
        smach.StateMachine.add('BAR', ExampleState(), {'done':'BAZ'})
        smach.StateMachine.add('BAZ',
                               ExampleState(),
                               {'done':'succeeded'})

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
