import cv2
import map
import numpy as np
from random import *
import math

T = map.Treasure()
img_address = "./origin_map/origin.jpg"  # 路径
randomm = T.DrawTreasure(img_address)  # 返回的画好的图像
# cv2.imshow("origin", T.origin_map)  # 原始图像
cv2.imshow("random", randomm)  # 画好随机位置的图像


# 随机resize
resize_randomm = cv2.resize(randomm, (randint(200, 200),randint(200,2000)))
# cv2.imshow("resize", resize_randomm)  #
img,loc= T.FindTreasure(resize_randomm)
#
# cv2.imshow("0",img)
print(loc)
cv2.waitKey(0)






"""
下面是检测的具体算法
V0.0.1: 赶着吃饭，只使用了面积+xy轴长差距，且并未封装到面向对象中
"""
# gray = cv2.cvtColor(resize_randomm, cv2.COLOR_BGR2GRAY)
# ret,thresh = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
# contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#contours为轮廓集，可以计算轮廓的长度、面积等
# for cnt in contours:
#     if len(cnt)>5: # 点的数量够多，滤噪点
#         S1=cv2.contourArea(cnt) # 封闭曲面的面积
#         ell=cv2.fitEllipse(cnt)
#         """
#         返回值：ellipse = [ (x, y) , (a, b), angle ]
#         （x, y）代表椭圆中心点的位置
#         （a, b）代表长短轴长度，应注意a、b为长短轴的直径，而非半径
#         angle 代表了中心旋转的角度
#         """
#         contour = cnt.reshape(-1, 2)
#         # 找出轮廓坐标中x的最大值
#         max_x = np.max(contour[:, 0])
#         # 找出轮廓坐标中x的最小值
#         min_x = np.min(contour[:, 0])
#         # 找出轮廓坐标中y的最大值
#         max_y = np.max(contour[:, 1])
#         # 找出轮廓坐标中y的最大值
#         min_y = np.min(contour[:, 1])
#
#         #计算轮廓最大xy周长 （即外接矩形长宽）
#         contour_x = max_x - min_x
#         contour_y = max_y - min_y
#         # S2 =math.pi*ell[1][0]*ell[1][1]
#         size = thresh.shape  # 读取图像的尺寸
#         length = size[1]  # 图像长度
#         height = size[0]  # 图像高度
#         block_x = length / 10
#         block_y = height / 10
#         block_s = (length * height) / 100  #每一个小方块的面积
#         # S=π(圆周率)×a×b  ab的比上小方块边长约为0.39，即“半径”为0.18 比值为 (π * 0.18 * 0.18 )/ 1 算出为0.102 取0.09（放宽下限）
#         min = 0.09 * block_s
#         max = 0.2 * block_s  #随便给的
#         if (min < S1 < max) and \
#                 (0.15 * block_x < contour_x < 0.5 * block_x ) and \
#                 (0.15 * block_y < contour_y < 0.5 * block_y ): # 判断是否满足条件 分别是面积 xy轴长 后面两个直接写里面了 想吃饭没时间弄
#             img = cv2.ellipse(resize_randomm, ell, (0, 255, 0), 2)
#     else:
#         img = resize_randomm
#             # print(str(S1) + "    " + str(S2)+"   "+str(ell[0][0])+"   "+str(ell[0][1]))
