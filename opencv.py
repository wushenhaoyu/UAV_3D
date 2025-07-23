#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from pyzbar.pyzbar import decode


def detect_qr_code():
    cap = cv2.VideoCapture(0)  # 打开摄像头，0表示默认摄像头
    while True:
        ret, frame = cap.read()  # 读取一帧图像
        if not ret:
            print("Failed to grab frame")
            break

        # 裁剪图像宽度一半，以中心为基准
        height, width = frame.shape[:2]
        start_x = width // 4  # 起始x坐标
        end_x = start_x + width // 2  # 结束x坐标
        cropped_frame = frame[:, start_x:end_x]

        decoded_objects = decode(cropped_frame)  # 解码二维码
        for obj in decoded_objects:
            print("QR Code Data:", obj.data.decode("utf-8"))
            # 在裁剪后的图像上绘制二维码的边界框
            points = obj.polygon
            if len(points) > 4:
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points
            n = len(hull)
            for j in range(0, n):
                cv2.line(cropped_frame, hull[j], hull[(j + 1) % n], (255, 0, 0), 3)

        cv2.imshow("QR Code Detection", cropped_frame)  # 显示裁剪后的图像
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 按下q键退出
            break

    cap.release()  # 释放摄像头资源
    cv2.destroyAllWindows()  # 关闭所有窗口

def detect_circle():
    cap = cv2.VideoCapture(0)  # 打开摄像头，0表示默认摄像头
    ret, frame = cap.read()  # 读取一帧图像
    if not ret:
        print("Failed to grab frame")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 2)
    circles = cv2.HoughCircles(
    gray,
    cv2.HOUGH_GRADIENT,
    dp=1.2,
    minDist=50,
    param1=40,
    param2=80,
    minRadius=0,
    maxRadius=500)
    out_img = frame.copy()
    circle_list = []
    if circles is not None:
        circles = np.round(circles[0, :]).astype(int)
        for (x, y, r) in circles:
            cv2.circle(out_img, (x, y), r, (0, 255, 0), 6)
            cv2.circle(out_img, (x, y), 2, (0, 0, 255), -1)  # 圓心
            circle_list.append((int(x), int(y), int(r)))
    
    cv2.imshow("a", out_img)
    cv2.waitKey()
    cv2.destroyAllWindows()
        

def detect_trangle():
    cap = cv2.VideoCapture(0)
    _, frame = cap.read()
    



if __name__ == '__main__':
    # detect_qr_code()
    detect_circle()