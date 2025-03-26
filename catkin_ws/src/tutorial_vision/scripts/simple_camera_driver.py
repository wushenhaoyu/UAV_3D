#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def main(argv):
    rospy.init_node("simple_camera_driver", argv=argv)
    camera_id = rospy.get_param("~camera_id", 0)
    frame_id = rospy.get_param("~frame_id", "mono_camera")
    rate_duration = rospy.get_param("~rate", 10)
    
    image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)
    capture = cv2.VideoCapture(camera_id)
    bridge = CvBridge()
    rate = rospy.Rate(rate_duration)
    while not rospy.is_shutdown():
        ret, frame = capture.read()
        if ret:
            try:
                image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.frame_id = frame_id
                image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(e)
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)

