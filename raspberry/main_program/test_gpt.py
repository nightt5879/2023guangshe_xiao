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
        adaptive_thresholded = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, blockSize, C)

        # 灰度化并应用高斯滤波
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # OTSU阈值化
        _, otsu_thresholded = cv2.threshold(gray_blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # 固定阈值化
        _, fixed_thresholded = cv2.threshold(gray_blurred, 95, 255, cv2.THRESH_BINARY)

        # 灰度化遮罩后的图像并应用高斯滤波
        masked_gray = cv2.bitwise_and(gray_blurred, gray_blurred, mask=mask)
        masked_gray_blurred = cv2.GaussianBlur(masked_gray, (5, 5), 0)

        # 遮罩后的灰度图像的OTSU阈值化
        _, masked_otsu_thresholded = cv2.threshold(masked_gray_blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # 遮罩后的灰度图像的固定阈值化
        _, masked_fixed_thresholded = cv2.threshold(masked_gray_blurred, 95, 255, cv2.THRESH_BINARY)

        # 显示原始图像
        cv2.imshow("Original Image", frame)

        # 显示HSV过滤后的图像
        cv2.imshow("HSV Filtered", mask)

        # 显示灰度图像
        cv2.imshow("Gray Image", gray_blurred)

        # 显示OTSU阈值化后的图像
        cv2.imshow("OTSU Thresholded Image", otsu_thresholded)

        # 显示固定阈值化后的图像
        cv2.imshow("Fixed Thresholded Image", fixed_thresholded)

        # 显示自适应阈值化后的图像
        cv2.imshow("Adaptive Thresholded Image", adaptive_thresholded)

        # 显示遮罩后的灰度图像
        cv2.imshow("Masked Gray Image", masked_gray_blurred)

        # 显示遮罩后的OTSU阈值化图像
        cv2.imshow("Masked OTSU Thresholded", masked_otsu_thresholded)

        # 显示遮罩后的固定阈值化图像
        cv2.imshow("Masked Fixed Thresholded", masked_fixed_thresholded)

        # 按'q'键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源并关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
