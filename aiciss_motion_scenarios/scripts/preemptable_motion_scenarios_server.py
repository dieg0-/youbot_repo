#!/usr/bin/env python

import rospy
import roslib
import smach
import smach_ros
import time
import sys
import math

from geometry_msgs.msg import Twist
from aiciss_motion_scenarios.srv import *
import actionlib
from preemptable_state import PreemptableState
import aiciss_motion_scenarios.msg

SERVER_NODE = "motion_scenarios_server"
SERVICE_NAME = "select_scenario"
KILLER_SERVICE_NAME = "killer_service"

_feedback = aiciss_motion_scenarios.msg.motion_scenariosFeedback()
_result   = aiciss_motion_scenarios.msg.motion_scenariosResult()

RUN_FLAG = True
########### State Machine Declaration ########### 
sm = smach.StateMachine(outcomes=['preempted'])
sm.userdata.sm_counter = 0
#################################################

############## Publisher Declaration ############      
pub = rospy.Publisher('cmd_vel', Twist)
twist = Twist()
linear_vel = 0.1
angular_vel = 0.2
circle_time = math.pi / linear_vel
triangle_time = 10.0
triangle_vel_y = math.sqrt(1.25) / triangle_time 
triangle_vel_x = 0.5 / triangle_time
#################################################

def initState(userdata):
	if userdata.init_counter_in == 0:
		time.sleep(3)
		return 'init'
	if userdata.init_counter_in == 1:
		time.sleep(3)
		return 'square'
	elif userdata.init_counter_in == 2:
		sm.userdata.sm_counter = 2
                time.sleep(3)
		return 'circle'
	elif userdata.init_counter_in == 3:
		sm.userdata.sm_counter = 3
		time.sleep(3)
		return 'triangle'
	elif userdata.init_counter_in == 4:
		stop()
		return 'init'
	#else:
	#	stop()
	#	return 'finish'

def square(userdata):
	rospy.loginfo('Execute state Square')
	if userdata.square_counter_in == 1:
		do_square()
		return 'square'
	elif userdata.square_counter_in == 2:
		sm.userdata.sm_counter = 2
                time.sleep(3)
		return 'circle'
	elif userdata.square_counter_in == 3:
		sm.userdata.sm_counter = 3
		time.sleep(3)
		return 'triangle'
	elif userdata.square_counter_in == 4:
		stop()
		return 'init'
	#else:
	#	stop()
	#	return 'finish'
		#return 'init'

def circle(userdata):
	rospy.loginfo('Execute state Circle')
	if userdata.circle_counter_in == 1:
		sm.userdata.sm_counter = 1
		time.sleep(3)
		return 'square'
	elif userdata.circle_counter_in == 2:
		do_circle()
		return 'circle'
	elif userdata.circle_counter_in == 3:
		sm.userdata.sm_counter = 3
		time.sleep(3)
		return 'triangle'
	elif userdata.circle_counter_in == 4:
		stop()
		return 'init'
	#else:
	#	stop()
	#	return 'finish'
		#return 'init'

def triangle(userdata):
	rospy.loginfo('Execute state Triangle')
	if userdata.triangle_counter_in == 1:
		sm.userdata.sm_counter = 1
		time.sleep(3)
		return 'square'
	elif userdata.triangle_counter_in == 2:
		sm.userdata.sm_counter = 2
		time.sleep(3)
		return 'circle'
	elif userdata.triangle_counter_in == 3:
		do_triangle()
		return 'triangle'
	elif userdata.triangle_counter_in == 4:
		stop()
		return 'init'
	#else:
	#	stop()
	#	return 'finish'
		#return 'init'

'''
Publish to /cmd_vel
  xl, xl and zl are linear velocities
  xa, ya and za are angular velocities
  time_sleep is the duration of the motion command
'''
def publish_velocity(xl, yl, zl, xa, ya, za, time_sleep):
	twist.linear.x = xl
	twist.linear.y = yl
	twist.linear.z = zl
	twist.angular.x = xa
	twist.angular.y = ya
	twist.angular.z = za
	
	pub.publish(twist)
	rospy.sleep(time_sleep)

'''
Publish 0 to /cmd_vel
'''
def stop():
	twist.linear.x = 0
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0
	
	pub.publish(twist)

'''
The initial state simply waits for commands for a state transition.
'''
def do_init():
	print "Waiting for commands..."

'''
Performs squared motion with predefined velocities and times.
'''
def do_square():
	print "-- Drawing line 1 --"
	publish_velocity(linear_vel, 0, 0, 0, 0, 0, 10)
	print "-- Drawing line 2 --"	
	publish_velocity(0, linear_vel, 0, 0, 0, 0, 10)
	print "-- Drawing line 3 --"	
	publish_velocity(- linear_vel, 0, 0, 0, 0, 0, 10)
	print "-- Drawing line 4 --"	
	publish_velocity(0, - linear_vel, 0, 0, 0, 0, 10)
	stop()

'''
Performs circular motion with predefined velocities and times.
'''
def do_circle():
	print "-- Drawing arc --"
	publish_velocity(linear_vel, 0, 0, 0, 0, angular_vel, circle_time)
	stop()

'''
Performs triangular motion with predefined velocities and times.
'''
def do_triangle():
	print "-- Drawing line 1 --"
	publish_velocity(linear_vel, 0, 0, 0, 0, 0, 10)
	#publish_velocity(0, 0, 0, 0, 0, angular_vel, triangle_time)
	print "-- Drawing line 2 --"
	publish_velocity(- triangle_vel_x, triangle_vel_y, 0, 0, 0, 0, triangle_time)
	#publish_velocity(0, 0, 0, 0, 0, angular_vel, triangle_time)
	print "-- Drawing line 3 --"
	publish_velocity(- triangle_vel_x, - triangle_vel_y, 0, 0, 0, 0, triangle_time)
	#publish_velocity(0, 0, 0, 0, 0, angular_vel, triangle_time)
	stop()

'''
Callback function for the service call.
Defines the next state of the state machine.
'''
def select_motion_callback(request):
	print "Receiving request [Scenario = %s]"%(request.a)
	sm.userdata.sm_counter = request.a
	return TestSrvResponse(request.a)

def killer_callback(request):
	print "Receiving kill request"
	RUN_FLAG = False
	return TestSrvResponse(request.a)

'''
Wait for the user inputs to select the next state of the state machine.
Runs the node.
'''
def select_motion_scenario():
	rospy.init_node(SERVER_NODE)
	#service = rospy.Service(SERVICE_NAME, TestSrv, select_motion_callback)
	################	
	_as = actionlib.SimpleActionServer('as', aiciss_motion_scenarios.msg.motion_scenariosAction, execute_cb = executeCb, auto_start = False)
	_as.start()
	################	
	killer_service = rospy.Service(KILLER_SERVICE_NAME, TestSrv, killer_callback)
	print "Offering service to select motion scenarios."
	run_state_machine()
	while not rospy.is_shutdown() and RUN_FLAG:			
		pass			
	#rospy.spin()

'''
Defines and initializes the state machine.
'''
def run_state_machine():
	with sm:
		smach.StateMachine.add('InitState', PreemptableState(initState, 
			input_keys=['init_counter_in'],
			output_keys=['init_counter_out'], 
			outcomes=['init', 'square', 'circle', 'triangle']),
			transitions={'init' : 'InitState', 'square' : 'Square', 'circle' : 'Circle', 'triangle' : 'Triangle'},
			remapping={'init_counter_in' : 'sm_counter', 'init_counter_out' : 'sm_counter'})

		smach.StateMachine.add('Square', PreemptableState(square, 
			input_keys=['square_counter_in'],
			output_keys=['square_counter_out'],
			outcomes=['square', 'circle', 'triangle', 'init']),
			transitions={'square' : 'Square', 'circle' : 'Circle', 'triangle' : 'Triangle', 'init' : 'InitState'},
			remapping={'square_counter_in' : 'sm_counter', 'square_counter_out' : 'sm_counter'})

		smach.StateMachine.add('Circle', PreemptableState(circle,
			input_keys=['circle_counter_in'],
			output_keys=['circle_counter_out'],
			outcomes=['square', 'circle', 'triangle', 'init']),
			transitions={'square' : 'Square', 'circle' : 'Circle', 'triangle' : 'Triangle', 'init' : 'InitState'},
			remapping={'circle_counter_in' : 'sm_counter'})

		smach.StateMachine.add('Triangle', PreemptableState(triangle,
			input_keys=['triangle_counter_in'],
			output_keys=['triangle_counter_out'],
			outcomes=['square', 'circle', 'triangle', 'init']),
			transitions={'square' : 'Square', 'circle' : 'Circle', 'triangle' : 'Triangle',  'init' : 'InitState'},
			remapping={'triangle_counter_in' : 'sm_counter'})

	outcome = sm.execute()

def executeCb(self, goal):
	print "Receiving request [Scenario = %s]"%(goal.order)
	if _as.is_preempt_requested():
		print "Preempting"
		_as.set_preempted()
        	success = False
	else:
		sm.userdata.sm_counter = goal.order

def main():
	select_motion_scenario()
	sys.exit()

if __name__ == "__main__":
	main()
	
	
