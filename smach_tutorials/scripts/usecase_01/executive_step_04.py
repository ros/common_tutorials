#!/usr/bin/env python
"""
Description:

Usage:
    $> roslaunch turtle_nodes.launch
    $> ./executive_step_04.py

Output:
    [INFO] : State machine starting in initial state 'RESET' with userdata: 
                []
    [INFO] : State machine transitioning 'RESET':'succeeded'-->'SPAWN'
    [INFO] : State machine transitioning 'SPAWN':'succeeded'-->'TELEPORT1'
    [INFO] : State machine transitioning 'TELEPORT1':'succeeded'-->'TELEPORT2'
    [INFO] : State machine transitioning 'TELEPORT2':'succeeded'-->'BIG'
    [WARN] : Still waiting for action server 'turtle_shape1' to start... is it running?
    [INFO] : Connected to action server 'turtle_shape1'.
    [INFO] 1279655938.783058: State machine transitioning 'BIG':'succeeded'-->'SMALL'
    [INFO] 1279655975.025202: State machine terminating 'SMALL':'succeeded':'succeeded'

"""

import roslib; roslib.load_manifest('smach_tutorials')
import rospy

import threading

import smach
from smach import StateMachine, ServiceState, SimpleActionState, IntrospectionServer

import std_srvs.srv
import turtlesim.srv
import turtle_actionlib.msg


def main():
    rospy.init_node('smach_usecase_step_04')

    # Construct static goals
    polygon_big = turtle_actionlib.msg.ShapeGoal(edges = 11, radius = 4.0)
    polygon_small = turtle_actionlib.msg.ShapeGoal(edges = 6, radius = 0.5) 

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Open the container
    with sm0:
        # Reset turtlesim
        StateMachine.add('RESET',
                ServiceState('reset', std_srvs.srv.Empty),
                {'succeeded':'SPAWN'})

        # Create a second turtle
        StateMachine.add('SPAWN',
                ServiceState('spawn', turtlesim.srv.Spawn,
                    request = turtlesim.srv.SpawnRequest(0.0,0.0,0.0,'turtle2')),
                {'succeeded':'TELEPORT1'})

        # Teleport turtle 1
        StateMachine.add('TELEPORT1',
                ServiceState('turtle1/teleport_absolute', turtlesim.srv.TeleportAbsolute,
                    request = turtlesim.srv.TeleportAbsoluteRequest(5.0,1.0,0.0)),
                {'succeeded':'TELEPORT2'})

        # Teleport turtle 2
        StateMachine.add('TELEPORT2',
                ServiceState('turtle2/teleport_absolute', turtlesim.srv.TeleportAbsolute,
                    request = turtlesim.srv.TeleportAbsoluteRequest(9.0,5.0,0.0)),
                {'succeeded':'BIG'})

        # Draw a large polygon with the first turtle
        StateMachine.add('BIG',
                SimpleActionState('turtle_shape1',turtle_actionlib.msg.ShapeAction,
                    goal = polygon_big),
                {'succeeded':'SMALL'})

        # Draw a small polygon with the second turtle
        StateMachine.add('SMALL',
                SimpleActionState('turtle_shape2',turtle_actionlib.msg.ShapeAction,
                    goal = polygon_small))

    # Attach a SMACH introspection server
    sis = IntrospectionServer('smach_usecase_01', sm0, '/USE_CASE')
    sis.start()

    # Set preempt handler
    smach.set_preempt_handler(sm0)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()

if __name__ == '__main__':
    main()
