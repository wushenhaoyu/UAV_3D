#!/usr/bin/env python
# coding: utf-8
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from tutorial_vision.msg import ObjectError
from cv_bridge import CvBridge, CvBridgeError


class CircleDetectorNode(object):
    def __init__(self):
        rospy.init_node('detect_circles_node', anonymous=True)

        self.bridge = CvBridge()
        # 原来的 error 话题
        self.pub_error = rospy.Publisher("/object_error", ObjectError, queue_size=10)
        # 新增的带标注图像话题
        self.pub_debug = rospy.Publisher("/camera/image_debug", Image, queue_size=1)

        rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        self.current_error = ObjectError()

    # ---------- detect_circles 保持不变 ----------
    @staticmethod
    def detect_circles(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 2)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=150,
            param2=50,
            minRadius=10,
            maxRadius=50
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            return circles[0, :]
        return []
    # --------------------------------------------

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        circles = self.detect_circles(frame)

        if len(circles) > 0:
            largest = max(circles, key=lambda c: c[2])  # 半径最大
            cx, cy, r = largest
            h, w = frame.shape[:2]

            # 计算并发布误差
            self.current_error.type = 2  # 2 代表圆形
            self.current_error.y = float(cx) - w / 2.0
            self.current_error.x = float(cy) - h / 2.0
            self.pub_error.publish(self.current_error)

            # --- 画圆、画中心点 ---
            cv2.circle(frame, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        else:
            self.current_error.x = 0
            self.current_error.y = 0

        # 无论有没有检测到，都画 error 文本
        text = "x_error: {:.1f}   y_error: {:.1f}".format(
            self.current_error.x, self.current_error.y)
        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # 发布带标注的图像
        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

    def spin(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        node = CircleDetectorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass