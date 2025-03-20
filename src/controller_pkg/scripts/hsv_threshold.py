#!/usr/bin/env python

import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Initialize ROS node (minimal setup for subscription)
rospy.init_node('hsv_threshold_script', anonymous=True)
bridge = CvBridge()
latest_image = None

def image_callback(data):
    global latest_image
    try:
        latest_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)

# Subscribe to the correct camera topic
rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, image_callback)

# Wait briefly for the first image
rospy.sleep(1)  # Give time for the subscriber to get an image

# Initial HSV values
lh = 0
ls = 0
lv = 50
uh = 180
us = 50
uv = 120
lower_hsv = np.array([lh, ls, lv])
upper_hsv = np.array([uh, us, uv])

# Setup window and trackbars
window_name = "HSV Calibrator"
cv.namedWindow(window_name)

def nothing(x):
    print("Trackbar value: " + str(x))
    pass

# Create trackbars
cv.createTrackbar('LowerH', window_name, lh, 255, nothing)
cv.createTrackbar('LowerS', window_name, ls, 255, nothing)
cv.createTrackbar('LowerV', window_name, lv, 255, nothing)
cv.createTrackbar('UpperH', window_name, uh, 255, nothing)
cv.createTrackbar('UpperS', window_name, us, 255, nothing)
cv.createTrackbar('UpperV', window_name, uv, 255, nothing)

font = cv.FONT_HERSHEY_SIMPLEX

print("Waiting for images from /B1/rrbot/camera1/image_raw...")

while not rospy.is_shutdown():
    if latest_image is None:
        rospy.loginfo("No image received yet, waiting...")
        rospy.sleep(0.1)
        continue

    # Process the latest image
    img = cv.medianBlur(latest_image, 5)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Update HSV values from trackbars
    lh = cv.getTrackbarPos('LowerH', window_name)
    uh = cv.getTrackbarPos('UpperH', window_name)
    ls = cv.getTrackbarPos('LowerS', window_name)
    us = cv.getTrackbarPos('UpperS', window_name)
    lv = cv.getTrackbarPos('LowerV', window_name)
    uv = cv.getTrackbarPos('UpperV', window_name)
    lower_hsv = np.array([lh, ls, lv])
    upper_hsv = np.array([uh, us, uv])

    # Threshold the HSV image and show it
    mask = cv.inRange(hsv, lower_hsv, upper_hsv)
    cv.putText(mask, 'Lower HSV: [' + str(lh) + ',' + str(ls) + ',' + str(lv) + ']', 
               (10, 30), font, 0.5, (200, 255, 155), 1, cv.LINE_AA)
    cv.putText(mask, 'Upper HSV: [' + str(uh) + ',' + str(us) + ',' + str(uv) + ']', 
               (10, 60), font, 0.5, (200, 255, 155), 1, cv.LINE_AA)

    cv.imshow(window_name, mask)

    # Exit on 'q' and print final values
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        print(f"Final Lower HSV: [{lh}, {ls}, {lv}]")
        print(f"Final Upper HSV: [{uh}, {us}, {uv}]")
        break

    rospy.sleep(0.1)  # Control loop rate

cv.destroyAllWindows()