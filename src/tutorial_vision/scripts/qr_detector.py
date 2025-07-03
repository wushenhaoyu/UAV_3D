#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from tutorial_vision.msg import StringStamped
from std_msgs.msg import Int32

camera_en = 0
def image_cb(msg, bridge, str_pub, img_pub):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # 1. 左右裁剪一半（宽度方向）
        height, width = cv_image.shape[:2]
        start_x = width // 3
        end_x   = start_x + width // 2
        cropped_image = cv_image[:, start_x:end_x]

        # 2. 上下再裁剪一半（高度方向）
        new_height = cropped_image.shape[0]
        start_y = new_height // 3
        end_y   = start_y + new_height // 2
        final_cropped_image = cropped_image[start_y:end_y, :]

        # 3. 解码二维码（基于裁剪图）
        decoded_objects = decode(final_cropped_image)

        # 4. 在裁剪图上绘制矩形框
        display_image = final_cropped_image.copy()
        qr_strings = []
        for obj in decoded_objects:
            (x, y, w, h) = obj.rect
            cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            qr_strings.append(obj.data.decode("utf-8"))

        # 5. 发布裁剪后的带框图像
        img_msg = bridge.cv2_to_imgmsg(display_image, "bgr8")
        img_msg.header = msg.header
        img_pub.publish(img_msg)

        # 6. 发布二维码字符串
        qr_msg = StringStamped()
        qr_msg.header = msg.header
        qr_msg.data = qr_strings
        str_pub.publish(qr_msg)

    except CvBridgeError as e:
        rospy.logerr(e)


def main(argv):
    rospy.init_node("qr_detector", argv=argv)
    bridge = CvBridge()
    str_pub = rospy.Publisher("qr_detect_result", StringStamped, queue_size=1)
    img_pub = rospy.Publisher("qr_cropped_with_box", Image, queue_size=1)

    rospy.Subscriber("camera/image_raw", Image,
                     lambda msg: image_cb(msg, bridge, str_pub, img_pub))
    
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
