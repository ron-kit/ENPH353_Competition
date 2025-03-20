#! /usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock  # Added for /clock
from cv_bridge import CvBridge

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower')
        
        # Publishers and Subscribers with corrected topics
        self.cmd_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.score_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.image_sub = rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)
        
        # OpenCV Bridge
        self.bridge = CvBridge()
        
        # Movement Command
        self.move = Twist()
        
        # Line Detection HSV Limits (adjust these as needed)
        self.low_H, self.high_H = 0, 180
        self.low_S, self.high_S = 0, 50
        self.low_V, self.high_V = 50, 120

        self.avg_x = None
        
        rospy.loginfo("Line Follower Node Initialized")
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]

        frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(frame_HSV, (self.low_H, self.low_S, self.low_V), 
                                     (self.high_H, self.high_S, self.high_V))
        
        roi_mask = mask[-int(height/4):, :]
        contours, _ = cv.findContours(roi_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if self.avg_x is None:
            self.avg_x = width // 2

        if contours:
            total_x = sum(cv.moments(cnt)['m10'] / cv.moments(cnt)['m00'] 
                          for cnt in contours if cv.moments(cnt)['m00'] > 0)
            self.avg_x = int(total_x / len(contours))
            error = self.avg_x - width // 2
            self.move.linear.x = 0.5
            self.move.angular.z = -error * 0.02
        else:
            self.move.linear.x = 0.0
            self.move.angular.z = 0.3
        
        self.cmd_pub.publish(self.move)

        cv.circle(frame, (self.avg_x, int(0.9*height)), 25, (0, 0, 255), -1)
        cv.imshow("Line Tracking", frame)
        # cv.imshow("Threshold", mask)
        processed_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(processed_image_msg)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User quit")

    def clock_callback(self, msg):
        self.score_pub.publish(int(msg.clock.to_sec()))

if __name__ == '__main__':
    try:
        LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv.destroyAllWindows()