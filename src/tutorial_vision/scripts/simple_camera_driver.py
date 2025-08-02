#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
from datetime import datetime
from tutorial_serial.msg import CameraEn, AninmalData
import os

class YOLOCameraNode:
    def __init__(self):
        # ----------- 固定参数 -----------
        self.model_path = "/home/flmg/UAV_3D/last.pt"
        self.confidence = 0.5
        self.camera_id = 0
        self.save_dir = "/home/flmg/Pictures/"

        self.cls2type = {
            "tiger":    0x02,
            "elephant": 0x03,
            "monkey":   0x04,
            "wolf":     0x05,
            "peacock":  0x06,
        }
        self.target_classes = list(self.cls2type.keys())

        # ----------- 初始化模型与摄像头 -----------
        print("Loading YOLO model...", flush=True)
        self.model = YOLO(self.model_path)
        self.capture = cv2.VideoCapture(self.camera_id)
        if not self.capture.isOpened():
            print(f"Fatal: Cannot open camera {self.camera_id}", flush=True)
            exit(1)

        # ----------- 初始化 ROS 组件 -----------
        self.bridge = CvBridge()
        self.pub_result = rospy.Publisher('yolo_result', AninmalData, queue_size=10)
        rospy.Subscriber('camera_en', CameraEn, self.enable_callback)

        print("YOLOCameraNode ready. Waiting for /camera_en = True", flush=True)

    # ----------- 触发预测处理 -----------
    def enable_callback(self, msg):
        x = msg.x
        y = msg.y

        print("Received enable signal, capturing...", flush=True)
        for _ in range(5):  # 清空摄像头缓存
            _, _ = self.capture.read()
        ret, frame = self.capture.read()
        if not ret:
            print("Camera frame capture failed.", flush=True)
            return

        # 裁剪图像区域
        x1, y1 = 200, 150
        x2, y2 = 400, 350
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        roi = frame[y1:y2, x1:x2]

        # 运行 YOLO 推理
        results = self.model.predict(roi, conf=self.confidence, verbose=False)
        names = self.model.names

        # 收集所有目标及其置信度
        detections = []
        for r in results:
            boxes = r.boxes
            for cls_id, conf in zip(boxes.cls, boxes.conf):
                cls_name = names[int(cls_id)]
                if cls_name in self.target_classes:
                    detections.append((cls_name, float(conf)))

        # 按置信度排序（高 → 低）
        detections.sort(key=lambda x: x[1], reverse=True)

        # 选择最多两个不同类别的目标（每类一个）
        selected = {}
        for cls_name, conf in detections:
            if cls_name not in selected:
                selected[cls_name] = conf
            if len(selected) >= 2:
                break

        # 如果识别出目标，则保存检测图像
        #out_frame = results[0].plot()
        #ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        #save_path = os.path.join(self.save_dir, f"{x}{y}.jpg")
        #cv2.imwrite(save_path, out_frame)
        #print(f"Saved result to {save_path}", flush=True)

        # 发布结果（每个目标单独发）
        for cls_name, conf in selected.items():
            msg_out = AninmalData()
            msg_out.type = self.cls2type[cls_name]
            msg_out.x = x
            msg_out.y = y
            msg_out.number = 1
            self.pub_result.publish(msg_out)
            print(f"Published: {cls_name}, conf: {conf:.2f}, x: {x}, y: {y}", flush=True)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('yolo_camera_node')
    node = YOLOCameraNode()
    node.run()

if __name__ == '__main__':
    main()
