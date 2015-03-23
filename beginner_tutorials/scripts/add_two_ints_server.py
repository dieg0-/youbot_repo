#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    # Declar the node
    rospy.init_node('add_two_ints_server')
    # Declare the service with AddTwoInts type.
    # Requests are passed to handle_add_two_ints function (called with instances of AddTwoIntsRequest and returns instances
    #  of AddTwoIntsResponse)
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    # Keeps the code from exiting until the service is shutdown.
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
