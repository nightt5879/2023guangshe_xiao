import cv2
import numpy as np

def main():
    # 打开默认摄像头
    cap = cv2.VideoCapture(0)
    # 设置成320乘以240的摄像头窗口
    cap.set(3, 320)
    cap.set(4, 240)


    # 设置黑色的HSV阈值范围
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 80])

    while True:
        # 读取一帧图像
        # Cam.ReadImg(0, 320, 0, 200) 读取0到320 和0 到200 的图像

        ret, frame = cap.read()
        # 剪切图像变成 0到320 和 0到200 的图像
        frame = frame[0:200, 0:320]
        if not ret:
            print("无法读取图像")
            break

        # 输入图像高斯滤波
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 使用HSV阈值过滤黑色
        mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.dilate(mask, kernel, iterations=3)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.bitwise_not(mask)  # 反转一下图像
        # 灰度化并应用高斯滤波
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # 灰度化遮罩后的图像并应用高斯滤波
        masked_gray = cv2.bitwise_and(gray_blurred, gray_blurred, mask=mask)
        masked_gray_blurred = cv2.GaussianBlur(masked_gray, (5, 5), 0)


        # 高斯滤波
        blur = cv2.GaussianBlur(mask, (5, 5), 0)

        # 高斯自适应阈值化
        blockSize = 31
        C = 5
        adaptive_thresholded = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, blockSize, C)

        # OTSU阈值化
        _, otsu_thresholded = cv2.threshold(gray_blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # 固定阈值化
        _, fixed_thresholded = cv2.threshold(gray_blurred, 95, 255, cv2.THRESH_BINARY)



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
