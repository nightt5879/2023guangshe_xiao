import cv2
import numpy as np


def find_squares(map_img):
    img = map_img
    sum = 0
    squares = []
    block_list = []
    w, h = img.shape[:-1]
    # 这张图像的总大小
    img_area = w * h
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 3)
    bin = cv2.Canny(blur, 10, 30, apertureSize=3)
    contours, _hierarchy = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print("轮廓数量：%d" % len(contours))
    # 轮廓遍历
    for cnt in contours:
        cnt_len = cv2.arcLength(cnt, True)  # 计算轮廓周长
        area = cv2.contourArea(cnt)
        cnt = cv2.approxPolyDP(cnt, 0.1 * cnt_len, True)  # 多边形逼近
        # 条件判断逼近边的数量是否为4，轮廓面积是否大于1000，检测轮廓是否为凸的
        if len(cnt) == 4 and cv2.isContourConvex(cnt) and area > 0.001 * img_area:
            cnt = cnt.reshape(-1, 2)
            squares.append(cnt)
            sum += 1
    output_img = cv2.drawContours(img,  squares, -1, (0, 255, 0), 2)
    print("我找到了方块数量：",sum,squares)
    return bin,blur,output_img


# img_address = "./origin_map/find.png"  # 路径
# img = cv2.imread(img_address)
# full_map,gray,blur,outputt = get_full_map(img)
# _,him = find_squares(img)
#
# cv2.imshow("ori_img", img)
# # cv2.imshow("gray", gray)
# cv2.imshow("blur",blur)
# cv2.imshow("edges", full_map)
# cv2.imshow("output", outputt)
# cv2.imshow("him", him)
# cv2.waitKey(0)  #按数字0就是前进1帧
cap = cv2.VideoCapture("output.avi")
while True:
    success, img = cap.read()
    cv2.imshow("origin", img)
    full_map,blur,outputt = find_squares(img)

    cv2.imshow("edges", full_map)
    cv2.imshow("blur",blur)
    cv2.imshow("draw", outputt)
    cv2.waitKey(0)  #按数字0就是前进1帧


