#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class ColorExtract(object):
    def __init__(self):
        self._red_pub = rospy.Publisher('red_image', Image, queue_size=1)
        self._detect_pub = rospy.Publisher('detect_image', Image, queue_size=1)
        self._image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self._bridge = CvBridge()

    def get_colored_area(self, cv_image, lower, upper):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower, upper)

        # Extract coutour
        image, contours, hierarchy = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=mask_image)

        area = cv2.countNonZero(mask_image)
        return (area, extracted_image, contours)
        
    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e

        # Detect red color
        red_area, red_image, contours = self.get_colored_area(cv_image, np.array([0,120,120]), np.array([10,255,255]))
        # Draw contours to input image
        #detect_image = cv2.drawContours(cv_image, contours, -1, (0,255,0))
        # Draw rectangle
        for i in range(len(contours)):
            x, y, w, h = cv2.boundingRect(contours[i])
            if w > 10 and h > 10: # Remove too small rectangle
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0))
            
        try:
            self._red_pub.publish(self._bridge.cv2_to_imgmsg(red_image, 'bgr8'))
            self._detect_pub.publish(self._bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError, e:
            print e
        rospy.loginfo('red pixel = %d' % (red_area))


if __name__ == '__main__':
    rospy.init_node('color_extract')
    color = ColorExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass