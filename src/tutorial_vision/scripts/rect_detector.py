#!/usr/bin/env python
# coding: utf-8
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from tutorial_vision.msg import ObjectError
from cv_bridge import CvBridge, CvBridgeError


class RectDetectorNode(object):
    def __init__(self):
        rospy.init_node('detect_squares_node', anonymous=True)

        self.bridge = CvBridge()

        # 原来的 error 话题
        self.pub_error = rospy.Publisher("/object_error", ObjectError, queue_size=10)
        # 新增的带标注图像话题
        self.pub_debug = rospy.Publisher("/camera/image_debug", Image, queue_size=1)

        rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        self.current_error = ObjectError()

    # ---------- 你的 detect_squares 原封不动 ----------
    @staticmethod
    def detect_squares(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 125, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        squares = []
        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            if len(approx) == 4:
                rect = cv2.minAreaRect(approx)
                w, h = rect[1]
                if min(w, h) < 1:
                    continue
                aspect = max(w, h) / min(w, h)
                area_cnt = cv2.contourArea(approx)
                area_rect = w * h
                area_ratio = area_cnt / area_rect if area_rect else 0
                if (0.8 <= aspect <= 1.2 and 
                    0.7 <= area_ratio <= 1.2 and
                    area_cnt > 75):
                    squares.append(approx)
        return squares
    # --------------------------------------------------

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        squares = self.detect_squares(frame)

        if squares:
            biggest = max(squares, key=cv2.contourArea)
            cx = int(np.mean(biggest[:, 0, 0]))
            cy = int(np.mean(biggest[:, 0, 1]))
            h, w = frame.shape[:2]
            self.current_error.type = 1
            x = cx - w / 2.0
            y = cy - h / 2.0
            self.current_error.x =  y
            self.current_error.y =  x
            self.pub_error.publish(self.current_error)

            # --- 画框、画中心点 ---
            cv2.drawContours(frame, [biggest], -1, (0, 255, 0), 2)
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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        node = RectDetectorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass