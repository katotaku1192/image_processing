#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray
from cv_bridge import CvBridge, CvBridgeError


class ArucoExtract(object):
    def __init__(self):
        self._detect_pub = rospy.Publisher('detect_image', Image, queue_size=1)
        self._point_pub = rospy.Publisher('ar_point', UInt16MultiArray, queue_size=1)
        self._image_sub = rospy.Subscriber('/zed_node/left/image_rect_gray', Image, self.callback)
        self._bridge = CvBridge()

        self.aruco = cv2.aruco
        self.dictionary = self.aruco.getPredefinedDictionary(self.aruco.DICT_4X4_50)


    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print e


        corners, ids, rejectedImgPoints = self.aruco.detectMarkers(cv_image, self.dictionary)
        detect_image = self.aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)

        ar_point = UInt16MultiArray()

        if corners:
            cx, cy = corners[0][0][2][0], corners[0][0][2][1]
            rx, ry = int(cx), int(cy)+10
            ar_point.data = [rx, ry]

            font = cv2.FONT_HERSHEY_PLAIN
            cv2.circle(detect_image,(rx, ry), 4, (0, 255, 0), -1)
            cv2.putText(detect_image, '('+str(rx)+', '+str(ry)+')', (rx, ry), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

        else:
            ar_point.data = [0, 0]

        #rospy.loginfo(ar_point.data)


        try:
            self._detect_pub.publish(self._bridge.cv2_to_imgmsg(detect_image, 'bgr8'))
            self._point_pub.publish(ar_point)
        except CvBridgeError, e:
            print e



if __name__ == '__main__':
    rospy.init_node('detect_aruco')
    det_ar = ArucoExtract()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass