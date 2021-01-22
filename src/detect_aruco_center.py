#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError


class ArucoExtract(object):
    def __init__(self):
        self._detect_pub_left = rospy.Publisher('detect_image_left', Image, queue_size=1)
        self._detect_pub_right = rospy.Publisher('detect_image_right', Image, queue_size=1)

        self._point_pub_left = rospy.Publisher('ar_point_left', Float32MultiArray, queue_size=1)
        self._point_pub_right = rospy.Publisher('ar_point_right', Float32MultiArray, queue_size=1)

        self._image_sub_left = rospy.Subscriber('/zed_node/left/image_rect_gray', Image, self.callbackLeft)
        self._image_sub_right = rospy.Subscriber('/zed_node/right/image_rect_gray', Image, self.callbackRight)
        
        self._bridge = CvBridge()

        self.aruco = cv2.aruco
        self.dictionary = self.aruco.getPredefinedDictionary(self.aruco.DICT_4X4_50)


    def callbackLeft(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e


        corners, ids, rejectedImgPoints = self.aruco.detectMarkers(cv_image, self.dictionary)
        detect_image = self.aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)

        ar_point = Float32MultiArray()

        # If find marker
        if corners:
            rx = (corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0]) / 4
            ry = (corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1]+corners[0][0][3][1]) / 4
            ar_point.data = [rx, ry]

            font = cv2.FONT_HERSHEY_PLAIN
            cv2.circle(detect_image,(int(rx), int(ry)), 4, (0, 0, 255), -1)
            cv2.putText(detect_image, '('+str(rx)+', '+str(ry)+')', (int(rx)+5, int(ry)-10), font, 1, (0, 0, 255), 1, cv2.LINE_AA)

        else:
            ar_point.data = [0, 0]

        #rospy.loginfo("Left : [%.2f , %.2f]", ar_point.data[0], ar_point.data[1])


        try:
            self._detect_pub_left.publish(self._bridge.cv2_to_imgmsg(detect_image, 'bgr8'))
            self._point_pub_left.publish(ar_point)
        except CvBridgeError, e:
            print e


    def callbackRight(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e


        corners, ids, rejectedImgPoints = self.aruco.detectMarkers(cv_image, self.dictionary)
        detect_image = self.aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)

        ar_point = Float32MultiArray()

        # If find marker
        if corners:
            rx = (corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0]) / 4
            ry = (corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1]+corners[0][0][3][1]) / 4
            ar_point.data = [rx, ry]

            font = cv2.FONT_HERSHEY_PLAIN
            cv2.circle(detect_image,(int(rx), int(ry)), 4, (0, 0, 255), -1)
            cv2.putText(detect_image, '('+str(rx)+', '+str(ry)+')', (int(rx)+5, int(ry)-10), font, 1, (0, 0, 255), 1, cv2.LINE_AA)

        else:
            ar_point.data = [0, 0]

        #rospy.loginfo("Right : [%.2f , %.2f]", ar_point.data[0], ar_point.data[1])


        try:
            self._detect_pub_right.publish(self._bridge.cv2_to_imgmsg(detect_image, 'bgr8'))
            self._point_pub_right.publish(ar_point)
        except CvBridgeError, e:
            print e



if __name__ == '__main__':
    rospy.init_node('detect_aruco_center')
    det_ar = ArucoExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass