#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from tutorial_vision.msg import StringStamped


def image_cb(msg, bridge, pub):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        decoded_objects = decode(cv_image)
        
        qr_msg = StringStamped()
        qr_msg.header = msg.header
        for obj in decoded_objects:
            qr_msg.data.append(obj.data.decode("utf-8"))
        pub.publish(qr_msg)
    except CvBridgeError as e:
        rospy.logerr(e)


def main(argv):
    rospy.init_node("qr_detector", argv=argv)
    bridge = CvBridge()
    str_pub = rospy.Publisher("qr_detect_result", StringStamped, queue_size=1)
    rospy.Subscriber("camera/image_raw", Image, lambda msg: image_cb(msg, bridge, str_pub))
    rospy.spin()
    
    
if __name__ == '__main__':
    main(sys.argv)

