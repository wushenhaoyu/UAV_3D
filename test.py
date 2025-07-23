import cv2
import numpy as np


def detect_squares(frame):
    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 二值化（反转：使黑色内部变为白色）
    _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
    # cv2.imshow("thresh", thresh)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # 查找轮廓（仅检测外部轮廓）
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    for cnt in contours:
        # 计算轮廓周长
        perimeter = cv2.arcLength(cnt, True)

        # 多边形近似（精度=周长的2%）
        approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)

        # 筛选四边形
        if len(approx) == 4:
            # 计算最小外接矩形
            rect = cv2.minAreaRect(approx)
            (w, h) = rect[1]

            # 计算宽高比
            aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 0

            # 计算轮廓面积与最小外接矩形面积比
            area_contour = cv2.contourArea(approx)
            area_rect = w * h
            area_ratio = area_contour / area_rect if area_rect > 0 else 0

            # 筛选条件：宽高比接近1，面积比>0.8，面积足够大
            if (0.8 <= aspect_ratio <= 1.1 and
                    0.7 <= area_ratio <= 1.0 and
                    area_contour > 500):  # 降低面积阈值以适应实时检测
                squares.append(approx)

    return squares

def begin_get_square():
    # 打开摄像头（0表示默认摄像头）
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("错误：无法打开摄像头")
        exit()
    while True:
        # 读取一帧
        ret, frame = cap.read()
        x1, y1, _ = frame.shape
        x1 /= 2
        y1 /= 2
        if not ret:
            print("错误：无法获取帧")
            break

        # 检测正方形
        squares = detect_squares(frame)

        # 在帧上绘制检测结果
        result = frame.copy()
        if len(squares) > 0:
            cv2.drawContours(result, squares, -1, (0, 255, 0), 3)
            # print(f"1-x:{squares[0][0][0][0]}, 1-y:{squares[0][0][0][1]}, "
            #       f"2-x:{squares[0][1][0][0]}, 2-y:{squares[0][1][0][1]}",
            #       f"3-x:{squares[0][2][0][0]}, 3-y:{squares[0][2][0][1]}",
            #       f"4-x:{squares[0][3][0][0]}, 4-y:{squares[0][3][0][1]}")
            x = 0
            y = 0
            for i in range(4):
                x += squares[0][i][0][0]
                y += squares[0][i][0][1]
            x /= 4
            y /= 4
            currentx = x - x1
            currenty = y - y1
            print(f"x:{currenty * -1}, y:{currentx * -1}")



        # 显示检测到的正方形数量
        cv2.putText(result, f"Squares: {len(squares)}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 显示结果
        cv2.imshow("Real-time Square Detection", result)

        # 按'q'键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    cap.release()

begin_get_square()
