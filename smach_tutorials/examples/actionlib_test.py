#!/usr/bin/env python
"""
Description:
    Spawn an actionlib action server, then create a state machine that sends
    some goals to the action server that will automatically succeed or abort.
    We expect the first goal to succeed, and the second goal to abort, so
    when the second goal aborts, we map that onto success of the state
    machine.

Usage:
    $> ./actionlib.py

Output:
    [INFO] : State machine starting in initial state 'GOAL_DEFAULT' with userdata: 
        []
    [WARN] : Still waiting for action server 'test_action' to start... is it running?
    [INFO] : State machine transitioning 'GOAL_DEFAULT':'succeeded'-->'GOAL_STATIC'
    [INFO] : State machine terminating 'GOAL_STATIC':'aborted':'succeeded'
"""

import rospy
import smach
import smach_ros

from smach_tutorials.msg import TestAction, TestGoal
from actionlib import *
from actionlib_msgs.msg import *

# Create a trivial action server
class TestServer:
    def __init__(self,name):
        self._sas = SimpleActionServer(name,
                TestAction,
                execute_cb=self.execute_cb)

    def execute_cb(self, msg):
        if msg.goal == 0:
            self._sas.set_succeeded()
        elif msg.goal == 1:
            self._sas.set_aborted()
        elif msg.goal == 2:
            self._sas.set_preempted()

def main():
    rospy.init_node('smach_example_actionlib')

    # Start an action server
    server = TestServer('test_action')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Open the container
    with sm0:
        # Add states to the container

        # Add a simple action sttate. This will use an emtpy, default goal
        # As seen in TestServer above, an empty goal will always return with
        # GoalStatus.SUCCEEDED, causing this simple action state to return
        # the outcome 'succeeded'
        smach.StateMachine.add('GOAL_DEFAULT',
                smach_ros.SimpleActionState('test_action', TestAction),
                {'succeeded':'GOAL_STATIC'})

        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.
        smach.StateMachine.add('GOAL_STATIC',
                smach_ros.SimpleActionState('test_action', TestAction,
                    goal = TestGoal(goal=1)),
                {'aborted':'succeeded'})

        # For more examples on how to set goals and process results, see 
        # executive_python/smach/tests/smach_actionlib.py

    # Execute SMACH plan
    outcome = sm0.execute()

    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()
