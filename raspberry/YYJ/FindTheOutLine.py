import cv2
import numpy as np

def GetOutLineAndRGB(img, CannyMin=100, CannyMax=150, Wmin=50, Wmax=200, Hmin=50, Hmax=200):
    imge = cv2.GaussianBlur(img, (5, 5), 0)  # 高斯滤波 平滑处理
    bin = cv2.Canny(imge, CannyMin, CannyMax, apertureSize=3)  # canny边缘检测  ps:这里是单通道的图像
    contours, _hierarchy = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 找轮廓 cv2.findContours
    # print("轮廓数量：%d" % len(contours))
    # 轮廓遍历
    area = []
    array = [0,0,0]
    for k in range(len(contours)):
        x, y, w, h = cv2.boundingRect(contours[k])  # 外接矩形
        if Wmin <= w <= Wmax and Hmin <= h <= Hmax:
            area.append(cv2.contourArea(contours[k]))
        else:
            ret = 0
    print(np.array(area))
    max =  np.argmax(np.array(area))  # 符合要求的最大矩形
    print(max)
    mask = cv2.drawContours(img, contours, max, 0, cv2.FILLED)
    # 画最框选出来的图形的中心点
    YRGB = int((y + h) / 2)
    XRGB = int((x + w) / 2)
    array = img[YRGB, XRGB]
    array = array[::-1]  # 翻转数组，使它为RGB排序
    # print(array)
    cv2.rectangle(img, (int(x + w * 3 / 8), int(y + h * 3 / 8)), (int(x + w * 5 / 8), int(y + h * 5 / 8)),(0, 255, 0), 2)
    return ret, array, bin

img = cv2.imread('C:/Users/nightt/Desktop/ploygon.jpg')
cv2.imshow('img',img)

cam = cv2.VideoCapture(1, cv2.CAP_DSHOW)
while True:
    ret, img = cam.read()
    inn, array, bin = GetOutLineAndRGB(img)
    print(inn)
    print(array)
    cv2.imshow('bin',bin)
    cv2.imshow('img',img)
    cv2.waitKey(1)
