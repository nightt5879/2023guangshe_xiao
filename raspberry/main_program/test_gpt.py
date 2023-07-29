import cv2
import numpy as np

def main():
    # 打开默认摄像头
    cap = cv2.VideoCapture(0)

    # 设置黑色的HSV阈值范围
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 46])

    while True:
        # 读取一帧图像
        ret, frame = cap.read()
        if not ret:
            print("无法读取图像")
            break

        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 使用HSV阈值过滤黑色
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # 高斯滤波
        blur = cv2.GaussianBlur(mask, (5, 5), 0)

        # 高斯自适应阈值化
        blockSize = 11
        C = 2
        thresholded = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, blockSize, C)

        # 显示原始图像
        cv2.imshow("Original Image", frame)

        # 显示HSV过滤后的图像
        cv2.imshow("HSV Filtered", mask)

        # 显示二值化后的图像
        cv2.imshow("Thresholded Image", thresholded)

        # 按'q'键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源并关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
