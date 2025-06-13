import cv2

# 打开默认的摄像头（通常是第一个摄像头）
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    # 捕获一帧图像
    ret, frame = cap.read()

    if not ret:
        print("无法获取帧")
        break

    # 显示图像
    cv2.imshow('1', frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) == ord('q'):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()