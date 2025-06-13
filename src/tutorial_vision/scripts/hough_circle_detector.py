#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tutorial_vision.msg import CircleInfo, CircleDetectResult


def image_cb(msg, param, bridge, pub, img_pub):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=param['dp'], minDist=param['minDist'],
            param1=param['param1'], param2=param['param2'], minRadius=param['minRadius'], maxRadius=param['maxRadius'])
            
        if circles is None:
            if img_pub is not None:
                img_pub.publish(msg)
            return
            
        circles = np.uint16(np.around(circles))
        results = CircleDetectResult()
        results.header = msg.header
        results.height = msg.height
        results.width = msg.width
        for i in circles[0, :]:
            circle_info = CircleInfo()
            circle_info.center_x = i[0]
            circle_info.center_y = i[1]
            circle_info.radius = i[2]
            results.circles.append(circle_info)
        pub.publish(results)
        
        if img_pub is not None:
            for i in circles[0, :]:
                cv2.circle(cv_image, (i[0], i[1]), 1, (0, 100, 100), 3)
                cv2.circle(cv_image, (i[0], i[1]), i[2], (255, 0, 255), 3)
            img_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        rospy.logerr(e)


def main(argv):
    rospy.init_node("hough_circle_detector", argv=argv)
    hough_param = {}
    hough_param['dp'] = rospy.get_param('~dp', 1)
    hough_param['minDist'] = rospy.get_param('~minDist', 20)
    hough_param['param1'] = rospy.get_param('~param1', 50)
    hough_param['param2'] = rospy.get_param('~param2', 30)
    hough_param['minRadius'] = rospy.get_param('~minRadius', 0)
    hough_param['maxRadius'] = rospy.get_param('~maxRadius', 0)
    bridge = CvBridge()
    img_pub = rospy.Publisher("circle_detect_result_img", Image, queue_size=1) if rospy.get_param('~pubImg', False) else None
    circle_pub = rospy.Publisher("circle_detect_result", CircleDetectResult, queue_size=1)
    rospy.Subscriber("camera/image_raw", Image,
        lambda msg: image_cb(msg, hough_param, bridge, circle_pub, img_pub)
    )
    rospy.spin()
    
    
if __name__ == '__main__':
    main(sys.argv)

