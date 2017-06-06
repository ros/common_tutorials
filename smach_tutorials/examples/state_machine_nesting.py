#!/usr/bin/env python
"""
Description:
    Create one state machine, put a state in it that sets something in the
    userdata key 'x'. Put another state machine inside of that state machine,
    tell it to fetch the userdata key 'x'. Put a state that reads 'x' into
    that nested state machine, and spew some info to rosout.

Usage:
    $> ./state_machine_nesting.py

Output:
    [INFO] : State machine starting in initial state 'SET' with userdata: 
        []
    [INFO] : State machine transitioning 'SET':'set_it'-->'NESTED'
    [INFO] : State machine starting in initial state 'GET' with userdata: 
        ['x']
    [INFO] : >>> GOT DATA! x = hello
    [INFO] : State machine terminating 'GET':'got_it':'done'
    [INFO] : State machine terminating 'NESTED':'done':'succeeded'

"""

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# Define a state to set some user data
class Setter(smach.State):
    def __init__(self, val):
        smach.State.__init__(self, outcomes = ['set_it'], output_keys = ['x'])
        self._val = val
    def execute(self, ud):
        # Set the data
        ud.x = self._val
        rospy.loginfo('>>> Set data: %s' % str(self._val))
        return 'set_it'

# Define a state to get some user data
class Getter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['got_it'], input_keys = ['x'])
    def execute(self, ud):
        # Wait for data to appear
        while 'x' not in ud:
            rospy.loginfo('>>> Waiting for data...')
            rospy.sleep(0.5)
        rospy.loginfo('>>> GOT DATA! x = '+str(ud.x))
        return 'got_it'

def main():
    rospy.init_node('smach_example_state_machine_nesting')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded'])

    # Open the container
    with sm0:
        # Add states to the container
        smach.StateMachine.add('SET', Setter(val='hello'), {'set_it':'NESTED'})

        sm1 = smach.StateMachine(outcomes=['done'], input_keys=['x'])
        smach.StateMachine.add('NESTED', sm1, {'done':'succeeded'})
        with sm1:
            smach.StateMachine.add('GET', Getter(), {'got_it':'done'})

    # Execute SMACH plan
    outcome = sm0.execute()

if __name__ == '__main__':
    main()
