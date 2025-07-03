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
        
        # 裁剪图像宽度一半，以中心为基准
        height, width = cv_image.shape[:2]
        start_x = width // 4  # 起始x坐标
        end_x = start_x + width // 2  # 结束x坐标
        cropped_image = cv_image[:, start_x:end_x]

        decoded_objects = decode(cropped_image)  # 解码二维码
        
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

