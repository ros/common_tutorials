#! /usr/bin/env python

# Copyright (c) 2010, Washington University in St. Louis
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Washington University in St. Louis nor the 
#       names of its contributors may be used to endorse or promote products 
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

import actionlib

import actionlib_tutorials.msg

class FibonacciAction(object):
	# create messages that are used to publish feedback/result
	_feedback = actionlib_tutorials.msg.FibonacciFeedback()
	_result   = actionlib_tutorials.msg.FibonacciResult()

	def __init__(self, name):
		self._action_name = name
		_as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start=False)
		_as.start()
		self._as = _as
		
	def execute_cb(self, goal):
		# helper variables
		r = rospy.Rate(1)
		success = True
		
		# append the seeds for the fibonacci sequence
		self._feedback.sequence = []
		self._feedback.sequence.append(0)
		self._feedback.sequence.append(1)
		
		# publish info to the console for the user
		rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
		
		# start executing the action
		for i in xrange(1, goal.order):
			# check that preempt has not been requested by the client
			if self._as.is_preempt_requested():
				rospy.loginfo('%s: Preempted' % self._action_name)
				self._as.set_preempted()
				success = False
				break
			self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
			# publish the feedback
			self._as.publish_feedback(self._feedback)
			# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
			r.sleep()
			
		if success:
			self._result.sequence = self._feedback.sequence
			rospy.loginfo('%s: Succeeded' % self._action_name)
			self._as.set_succeeded(self._result)
			
if __name__ == '__main__':
  rospy.init_node('fibonacci')
  FibonacciAction(rospy.get_name())
  rospy.spin()
