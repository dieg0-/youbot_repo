#!/usr/bin/env python

import sys
import rospy

from aiciss_motion_scenarios.srv import *

SERVICE_NAME = "select_scenario"

'''
Performs a request to the server (motion_scenarios_server.py),
the request being the next state (motion scenario) to be executed.
'''
def request_scenario(s):
	rospy.wait_for_service(SERVICE_NAME)
	try:
		select_scenario = rospy.ServiceProxy(SERVICE_NAME, TestSrv)
		response = select_scenario(s)
		return response.s
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def menu():
	answer = True
	while answer:
		print("Select a Motion Scenario to be executed:\n" +
			"1. Scenario 1 (Square)\n" +
			"2. Scenario 2 (Circle)\n" +
			"3. Scenario 3 (Triangle)\n" +
			"4. Pause.\n")

		answer = raw_input("Insert the number of the desired scenario:\n")
		s = 0
		if answer == "1":
			s = int(answer)
			print "Scenario 1 selected.\n"
			request_scenario(s)
		elif answer == "2":
			s = int(answer)
			print "Scenario 2 selected.\n"
			request_scenario(s)
		elif answer == "3":
			s = int(answer)
			print "Scenario 3 selected.\n"
			request_scenario(s)
		elif answer == "4":
			s = int(answer)
			request_scenario(s)
			print "Closing.\n"
			sys.exit()
		else:   
			s = 1
			print "Not a valid option. Scenario 1 selected by default.\n"
			request_scenario(s)
def main():
	print "Initialize Motion-Scenarios Client."
	menu()

if __name__ == "__main__":
	main()
