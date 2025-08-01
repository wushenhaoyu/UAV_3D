#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
from collections import Counter
from datetime import datetime
from tutorial_serial.msg import CameraEn , AninmalData
import os

class YOLOCameraNode:
    def __init__(self):
        # ----------- 固定参数 -----------
        self.model_path = "/home/flmg/UAV_3D/121.pt"
        self.confidence = 0.5
        self.camera_id  = 0
        self.save_dir   = "/home/flmg/Pictures/"

        self.cls2type = {
            "tiger":    0x02,
            "elephant": 0x03,
            "monkey":   0x04,
            "wolf":     0x05,
            "peacock":  0x06,
        }
        self.target_classes = list(self.cls2type.keys())

        # ----------- 初始化模型与摄像头 -----------
        rospy.loginfo("Loading YOLO model...")
        self.model = YOLO(self.model_path)
        self.capture = cv2.VideoCapture(self.camera_id)
        if not self.capture.isOpened():
            rospy.logfatal("Cannot open camera %d", self.camera_id)
            exit(1)

        # ----------- 初始化 ROS 组件 -----------
        self.bridge = CvBridge()
        self.pub_result = rospy.Publisher('yolo_result', AninmalData, queue_size=10)
        rospy.Subscriber('camera_en', CameraEn, self.enable_callback)

        rospy.loginfo("YOLOCameraNode ready. Waiting for /camera_en = True")

    # ----------- 触发预测处理 -----------
    def enable_callback(self, msg):
        x = msg.x
        y = msg.y

        print("Received enable signal, capturing...", flush=True)
        for i in range(5):
            _, _ = self.capture.read()
        ret, frame = self.capture.read()
        if not ret:
            print("Camera frame capture failed.", flush=True)
            return

        # 裁剪图像区域
        x1, y1 = 167, 100
        x2, y2 = 455, 300
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        roi = frame[y1:y2, x1:x2]

        # 运行 YOLO 推理
        results = self.model.predict(roi, conf=self.confidence, verbose=False)
        names = self.model.names
        counter = Counter()
        has_target = False

        for r in results:
            for cls_id in r.boxes.cls:
                cls_name = names[int(cls_id)]
                if cls_name in self.target_classes:
                    counter[cls_name] += 1
                    has_target = True

        top_two = counter.most_common(2)

        # 保存图像（有识别结果）
        if has_target:
            out_frame = results[0].plot()
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            save_path = os.path.join(self.save_dir, f"{ts}.jpg")
            cv2.imwrite(save_path, out_frame)
            print("Saved result to {save_path}", flush=True)

        # 发布识别结果
        for cls_name, cnt in top_two:
            msg_out = AninmalData()
            msg_out.type = self.cls2type[cls_name]
            msg_out.x = x
            msg_out.y = y
            msg_out.number = int(cnt)
            self.pub_result.publish(msg_out)
            print(f"Published: {cls_name} cnt: {cnt}", flush=True)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('yolo_camera_node')
    node = YOLOCameraNode()
    node.run()

if __name__ == '__main__':
    main()
