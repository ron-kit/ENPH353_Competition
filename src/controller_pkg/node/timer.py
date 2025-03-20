#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

class TimerNode:
    def __init__(self):
        rospy.init_node('timer_node')
        self.pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        rospy.sleep(0.1)  # Small delay here
        
        rospy.Subscriber('/clock', Clock, self.clock_callback)
        self.start_time = None
        rospy.loginfo("Timer Node Initialized")

        rospy.wait_for_message('/clock', Clock)
        # Start sequence
        self.pub.publish("Started timer!")
        rospy.loginfo("Timer started")
        rospy.sleep(1.0)  # Wait 1 second
        self.pub.publish("Stopped the timer!")
        rospy.loginfo("Timer stopped")
        rospy.signal_shutdown("Timer complete")

    def clock_callback(self, msg):
        pass

if __name__ == '__main__':
    try:
        TimerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass