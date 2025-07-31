import cv2
from ultralytics import YOLO
import time
import os

# 加载YOLO模型（比如 yolov8n.pt 或 yolov11n.pt）
model = YOLO("yolov8n.pt")  # 可换为你训练好的 best.pt 等

# 创建输出文件夹
os.makedirs("crops", exist_ok=True)

# 打开摄像头
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

count = 0  # 保存图片计数器

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取帧")
        break

    # YOLO 推理（自动转为RGB）
    results = model(frame)[0]

    for i, box in enumerate(results.boxes):
        # 提取坐标和置信度
        cls_id = int(box.cls)
        conf = float(box.conf)
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        # 绘制矩形框和标签
        label = f"{model.names[cls_id]} {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # 显示画面
    cv2.imshow("YOLO Detection", frame)

    # 按下 q 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
