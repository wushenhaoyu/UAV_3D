
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
        self.pub    = rospy.Publisher("/object_error", ObjectError, queue_size=10)
        rospy.Subscriber("/camera/image_raw", Image, self.img_cb)

        self.current_error = ObjectError()   


    @staticmethod
    def detect_squares(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        squares = []
        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            if len(approx) == 4:
                rect = cv2.minAreaRect(approx)
                w, h = rect[1]
                if min(w, h) < 1:  # 避免除 0
                    continue
                aspect = max(w, h) / min(w, h)
                area_cnt = cv2.contourArea(approx)
                area_rect = w * h
                area_ratio = area_cnt / area_rect if area_rect else 0
                if (0.8 <= aspect <= 1.1 and
                    0.7 <= area_ratio <= 1.0 and
                    area_cnt > 500):
                    squares.append(approx)
        return squares

    def img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        squares = self.detect_squares(frame)
        if squares:
            biggest = max(squares, key=cv2.contourArea)
            cx = np.mean(biggest[:, 0, 0])
            cy = np.mean(biggest[:, 0, 1])
            h, w = frame.shape[:2]
            self.current_error.type = 1 #1 为 正方形
            self.current_error.x = cx - w / 2.0
            self.current_error.y = cy - h / 2.0
            self.pub.publish(self.current_error)
        else:
            self.current_error.x = 0
            self.current_error.y = 0




    def spin(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        node = RectDetectorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass