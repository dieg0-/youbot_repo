#!/usr/bin/env python

#Need rospy if writing a Node
import rospy
from std_msgs.msg import String

def talker():
    # Declares that the node is publishing to a topic called 'chatter' messages of type String.
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # Tells rospy the namy of your node ("talker")
    rospy.init_node('talker', anonymous=True)
    # Create a 10 Hz rate object
    rate = rospy.Rate(10) # 10hz

    # Check is_shutdown to see if the program should exit, like with a Ctrl+C
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # Messages are printed to the screen, to the node's log file and to rosout.
        rospy.loginfo(hello_str)
        # Publish a String to chatter topic
        pub.publish(hello_str)
        # Sleeps long enough to maintain the 10 Hz rate
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
