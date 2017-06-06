#!/usr/bin/env python
"""
Description:

Usage:
    $> roslaunch turtle_nodes.launch
    $> ./executive_step_06.py

Output:
    [INFO] : State machine starting in initial state 'RESET' with userdata:
           
    [INFO] : State machine transitioning 'RESET':'succeeded'-->'SPAWN'
    [INFO] : State machine transitioning 'SPAWN':'succeeded'-->'TELEPORT1'
    [INFO] : State machine transitioning 'TELEPORT1':'succeeded'-->'TELEPORT2'
    [INFO] : State machine transitioning 'TELEPORT2':'succeeded'-->'DRAW_SHAPES'
    [INFO] : Concurrence starting with userdata:
        []
    [INFO] : State machine starting in initial state 'DRAW_WITH_MONITOR' with userdata:
        []
    [INFO] : Concurrence starting with userdata:
        []
    [WARN] : Still waiting for action server 'turtle_shape1' to start... is it running?
    [WARN] : Still waiting for action server 'turtle_shape2' to start... is it running?
    [INFO] : Connected to action server 'turtle_shape2'.
    [INFO] : Connected to action server 'turtle_shape1'.
    [INFO] : Preempt requested on action 'turtle_shape2'
    [INFO] : Preempt on action 'turtle_shape2' cancelling goal:
    edges: 6
    radius: 0.5
    [INFO] : Concurrent Outcomes: {'MONITOR': 'invalid', 'DRAW': 'preempted'}
    [INFO] : State machine transitioning 'DRAW_WITH_MONITOR':'interrupted'-->'WAIT_FOR_CLEAR'
    [INFO] : State machine transitioning 'WAIT_FOR_CLEAR':'invalid'-->'DRAW_WITH_MONITOR'
    [INFO] : Concurrence starting with userdata:
        []
    [INFO] : Concurrent Outcomes: {'MONITOR': 'preempted', 'DRAW': 'succeeded'}
    [INFO] : State machine terminating 'DRAW_WITH_MONITOR':'succeeded':'succeeded'
    [INFO] : Concurrent Outcomes: {'SMALL': 'succeeded', 'BIG': 'succeeded'}
    [INFO] : State machine terminating 'DRAW_SHAPES':'succeeded':'succeeded'

"""

import roslib; roslib.load_manifest('smach_tutorials')
import rospy

import threading
from math import sqrt, pow

import smach
from smach import StateMachine, ServiceState, SimpleActionState, MonitorState, IntrospectionServer, Concurrence

import std_srvs.srv
import turtlesim.srv
import turtlesim.msg
import turtle_actionlib.msg


def main():
    rospy.init_node('smach_usecase_step_06')

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
                {'succeeded':'DRAW_SHAPES'})

        # Draw some polygons
        shapes_cc = Concurrence(
                outcomes=['succeeded','aborted','preempted'],
                default_outcome='aborted',
                outcome_map = {'succeeded':{'BIG':'succeeded','SMALL':'succeeded'}})
        StateMachine.add('DRAW_SHAPES',shapes_cc)
        with shapes_cc:
            # Draw a large polygon with the first turtle
            Concurrence.add('BIG',
                    SimpleActionState('turtle_shape1',turtle_actionlib.msg.ShapeAction,
                        goal = polygon_big))
            # Draw a small polygon with the second turtle
            draw_monitor_cc = Concurrence(
                    ['succeeded','aborted','preempted'],
                    'aborted',
                    child_termination_cb = lambda so: True,
                    outcome_map = {
                        'succeeded':{'DRAW':'succeeded'},
                        'preempted':{'DRAW':'preempted','MONITOR':'preempted'},
                        'aborted':{'MONITOR':'invalid'}})
            Concurrence.add('SMALL',draw_monitor_cc)
            with draw_monitor_cc:
                Concurrence.add('DRAW',
                        SimpleActionState('turtle_shape2',turtle_actionlib.msg.ShapeAction,
                            goal = polygon_small))

                def turtle_far_away(ud, msg):
                    """Returns True while turtle pose in msg is at least 1 unit away from (9,5)"""
                    if sqrt(pow(msg.x-9.0,2) + pow(msg.y-5.0,2)) > 2.0:
                        return True
                    return False
                Concurrence.add('MONITOR',
                        MonitorState('/turtle1/pose',turtlesim.msg.Pose,
                            cond_cb = turtle_far_away))

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
