#!/usr/bin/env python
# coding: utf-8
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from tutorial_vision.msg import ObjectError
from cv_bridge import CvBridge, CvBridgeError


class TriangleDetectorNode(object):
    def __init__(self):
        rospy.init_node('detect_triangles_node', anonymous=True)

        self.bridge = CvBridge()

        # 原来的 error 话题
        self.pub_error = rospy.Publisher("/object_error", ObjectError, queue_size=10)
        # 新增的带标注图像话题
        self.pub_debug = rospy.Publisher("/camera/image_debug", Image, queue_size=1)

        rospy.Subscriber("/camera/image_raw", Image, self.img_cb)
        self.current_error = ObjectError()

    # ---------- 检测三角形 ----------
    @staticmethod
    def detect_triangles(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        triangles = []
        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

            if len(approx) == 3:  # 三角形
                pts = approx.reshape(3, 2)
                side_lengths = [
                    np.linalg.norm(pts[0] - pts[1]),
                    np.linalg.norm(pts[1] - pts[2]),
                    np.linalg.norm(pts[2] - pts[0])
                ]
                mean_len = np.mean(side_lengths)
                if mean_len < 8:  # 忽略太小的
                    continue
                # 判断是否接近等边三角形（每条边差异 < 15%）
                if max(side_lengths) / min(side_lengths) < 1.2:
                    area = cv2.contourArea(approx)
                    if area > 200:
                        triangles.append(approx)
        return triangles
    # ---------------------------------

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        triangles = self.detect_triangles(frame)

        if triangles:
            biggest = max(triangles, key=cv2.contourArea)
            # 计算质心
            M = cv2.moments(biggest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx = cy = 0

            h, w = frame.shape[:2]
            self.current_error.type = 3          # 3 代表三角形
            self.current_error.x = float(cx) - w / 2.0
            self.current_error.y = float(cy) - h / 2.0
            self.pub_error.publish(self.current_error)

            # --- 画轮廓、画中心点 ---
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
        node = TriangleDetectorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass