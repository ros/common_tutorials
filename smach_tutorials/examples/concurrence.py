#!/usr/bin/env python
"""
Description:
    This creates a concurrence with two states, one state, 'SET', waits three
    seconds before setting something in userdata, while another state 'GET'
    blocks while checking for thie userdata key, and only returns once it
    has been set.

Usage:
    $> ./concurrence.py

Output:
    [INFO] 1279226335.169182: Concurrence starting with userdata: 
            []
    [INFO] : >>> Waiting for data...
    [INFO] : >>> Waiting for data...
    [INFO] : >>> Waiting for data...
    [INFO] : >>> Waiting for data...
    [INFO] : >>> Waiting for data...
    [INFO] : >>> Waiting for data...
    [INFO] : >>> Set data: hello
    [INFO] : >>> GOT DATA! x = hello
    [INFO] : Concurrent Outcomes: {'SET': 'set_it', 'GET': 'got_it'}
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
        # Delay a bit to make this clear
        rospy.sleep(3.0)
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
    rospy.init_node('smach_example_concurrence')

    # Create a SMACH state machine
    cc0 = smach.Concurrence(
            outcomes=['succeeded', 'aborted'],
            default_outcome='aborted',
            outcome_map = {'succeeded':{'SET':'set_it','GET':'got_it'}})

    # Open the container
    with cc0:
        # Add states to the container
        smach.Concurrence.add('SET', Setter(val='hello'))
        smach.Concurrence.add('GET', Getter())

    # Execute SMACH plan
    outcome = cc0.execute()

if __name__ == '__main__':
    main()
