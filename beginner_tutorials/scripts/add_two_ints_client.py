#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

# !! For Clients you don't have to call init_node()

def add_two_ints_client(x, y):
    # Blocks until the service named add_two_ints is available.
    rospy.wait_for_service('add_two_ints')
    try:
        # Create a handle for calling the service, of type AddTwoInts
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        # Call the handle
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    #Arguments received in client's call: rosrun beginner_tutorials add_two_ints_client.py 4 5
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
