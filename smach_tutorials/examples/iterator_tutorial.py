#! /usr/bin/env python
## %Tag(FULLTEXT)%


import roslib; roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import rospy

import smach
from smach import Iterator, StateMachine, CBState
from smach_ros import ConditionState, IntrospectionServer

def construct_sm():
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])
    sm.userdata.nums = range(25, 88, 3)
    sm.userdata.even_nums = []
    sm.userdata.odd_nums = []
    with sm:
## %Tag(ITERATOR)%
        tutorial_it = Iterator(outcomes = ['succeeded','preempted','aborted'],
                               input_keys = ['nums', 'even_nums', 'odd_nums'],
                               it = lambda: range(0, len(sm.userdata.nums)),
                               output_keys = ['even_nums', 'odd_nums'],
                               it_label = 'index',
                               exhausted_outcome = 'succeeded')
## %EndTag(ITERATOR)% 
## %Tag(CONTAINER)%
        with tutorial_it:
            container_sm = StateMachine(outcomes = ['succeeded','preempted','aborted','continue'],
                                        input_keys = ['nums', 'index', 'even_nums', 'odd_nums'],
                                        output_keys = ['even_nums', 'odd_nums'])
            with container_sm:
                #test wether even or odd
                StateMachine.add('EVEN_OR_ODD',
                                 ConditionState(cond_cb = lambda ud:ud.nums[ud.index]%2, 
                                                input_keys=['nums', 'index']),
                                 {'true':'ODD',
                                  'false':'EVEN' })
                #add even state
                @smach.cb_interface(input_keys=['nums', 'index', 'even_nums'],
                                    output_keys=['odd_nums'], 
                                    outcomes=['succeeded'])
                def even_cb(ud):
                    ud.even_nums.append(ud.nums[ud.index])
                    return 'succeeded'
                StateMachine.add('EVEN', CBState(even_cb), 
                                 {'succeeded':'continue'})
                #add odd state
                @smach.cb_interface(input_keys=['nums', 'index', 'odd_nums'], 
                                    output_keys=['odd_nums'], 
                                    outcomes=['succeeded'])
                def odd_cb(ud):
                    ud.odd_nums.append(ud.nums[ud.index])
                    return 'succeeded'
                StateMachine.add('ODD', CBState(odd_cb), 
                                 {'succeeded':'continue'})
## %EndTag(CONTAINER)%
## %Tag(ADDCONTAINER)%
            #close container_sm
            Iterator.set_contained_state('CONTAINER_STATE', 
                                         container_sm, 
                                         loop_outcomes=['continue'])
## %EndTag(ADDCONTAINER)%
## %Tag(ADDITERATOR)%
        #close the tutorial_it
        StateMachine.add('TUTORIAL_IT',tutorial_it,
                     {'succeeded':'succeeded',
                      'aborted':'aborted'})
## %EndTag(ADDITERATOR)%
    return sm

def main():
    rospy.init_node("iterator_tutorial")
    sm_iterator = construct_sm()

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('iterator_tutorial',sm_iterator,'/ITERATOR_TUTORIAL')
    intro_server.start()


    outcome = sm_iterator.execute()

    rospy.spin()
    intro_server.stop()

if __name__ == "__main__":
    main()
## %EndTag(FULLTEXT)%  
