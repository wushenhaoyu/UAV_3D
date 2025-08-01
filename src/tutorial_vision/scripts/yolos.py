#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from tutorial_vision.msg import Aninmal
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from collections import Counter
from datetime import datetime
import os

class YOLOv8ROS:
    def __init__(self):
        # ---------------- 参数 ----------------
        self.model_path   = rospy.get_param('~model_path', '/home/flmg/UAV_3D/121.pt')
        self.confidence   = rospy.get_param('~confidence', 0.5)

        # 名字到 type 的映射
        self.cls2type = {
            "tiger":    0x02,
            "elephant": 0x03,
            "monkey":   0x04,
            "wolf":     0x05,
            "peacock":  0x06,
        }
        self.target_classes = list(self.cls2type.keys())

        # 模型加载
        self.model = YOLO(self.model_path)

        # ROS 通信
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('yolo_result', Aninmal, queue_size=10)
        rospy.Subscriber('camera/image_raw', Image, self.image_callback)

        rospy.loginfo("YOLOv8 ROS node started")

    # -------------------------------------------------

    def image_callback(self, msg):

        # 1. 图像转换
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        x1, y1 = 167, 84
        x2, y2 = 455, 323
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

    # 裁剪区域
        frame = frame[y1:y2, x1:x2] 

        # 2. 推理
        results = self.model.predict(frame,conf=self.confidence, verbose=False)

        # 3. 统计类别出现次数
        names = self.model.names
        counter = Counter()
        has_target = False
        for r in results:
            for cls_id in r.boxes.cls:
                cls_name = names[int(cls_id)]
                if cls_name in self.target_classes:
                    counter[cls_name] += 1
                    has_target = True

        # 4. 最多取两种动物
        top_two = counter.most_common(2)        # [('tiger', 3), ('elephant', 1)]

        # 5. 保存结果图（仅当检测到目标）
        if has_target:
            # 把框画到裁剪后的 roi 上
            plotted = results[0].plot()   # Ultralytics YOLO 自带画框
            # 拼接回原图坐标，方便调试
            out_frame =  plotted
            # 生成时间戳文件名
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            save_path = os.path.join("/home/flmg/Pictures/", f"{ts}.jpg")
            cv2.imwrite(save_path, out_frame)
            rospy.loginfo("Saved detection result to %s", save_path)

        # 5. 发布（最多两条消息）
        for cls_name, cnt in top_two:
            msg_out = Aninmal()
            msg_out.type   = self.cls2type[cls_name]
            msg_out.number = int(cnt)
            rospy.loginfo("Published: %s %d", cls_name, cnt)
            self.pub.publish(msg_out)


def main():
    rospy.init_node('yolov8_predict_node')
    YOLOv8ROS()
    rospy.spin()


if __name__ == '__main__':
    main()