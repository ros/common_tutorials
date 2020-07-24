#!/usr/bin/env python
"""
Description:
    Create a simple 3-state state sequence. A Sequence is a StateMachine
    that just assumes that states added in some sequence should be
    executed in the order in which they were added, but only if they
    return the "connector outcome" given in the Sequence constructor.

Usage:
    $> ./sequence.py

Output:
    [INFO] : State machine starting in initial state 'FOO' with userdata: 
            []
    [INFO] : State machine transitioning 'FOO':'done'-->'BAR'
    [INFO] : State machine transitioning 'BAR':'done'-->'BAZ'
    [INFO] : State machine terminating 'BAZ':'done':'succeeded'
"""

import rospy
import smach
import smach_ros

class ExampleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['done'])
    def execute(self, ud):
        return 'done'

def main():
    rospy.init_node('smach_example_sequence')

    # Create a SMACH state machine
    sq = smach.Sequence(
            outcomes = ['succeeded'],
            connector_outcome = 'done')

    # Open the container
    with sq:
        # Add states to the container
        smach.Sequence.add('FOO', ExampleState())
        smach.Sequence.add('BAR', ExampleState())
        smach.Sequence.add('BAZ', ExampleState(), {'done':'succeeded'})

    # Execute SMACH plan
    outcome = sq.execute()

if __name__ == '__main__':
    main()
