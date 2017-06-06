#!/usr/bin/env python
"""
Description:

Usage:
    $> ./executive_step_01.py

Output:
    [ERROR] : InvalidTransitionError: State machine failed consistency check: 
    No initial state set.

    Available states: []
    [ERROR] : Container consistency check failed.
    [ERROR] : InvalidTransitionError: State machine failed consistency check: 
    No initial state set.

    Available states: []
    [ERROR] : Container consistency check failed.
"""

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach

def main():
    rospy.init_node('smach_usecase_step_01')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=[])

    # Open the container
    with sm0:
        pass

    # Execute SMACH tree
    outcome = sm0.execute()

    # Signal ROS shutdown (kill threads in background)
    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()
