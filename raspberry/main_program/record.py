import cv2
import os
import move
import sys
from util.get_map import show_lcd
sys.path.insert(0, "/home/pi/.local/lib/python3.7/site-packages")
import os

os.chdir("/home/pi/Desktop/main_program")
# 指定保存图片的路径
save_path = '/home/pi/Desktop/main_program/data/red_3'
if not os.path.exists(save_path):
    os.makedirs(save_path)

# 打开默认摄像头
cap = cv2.VideoCapture(0)
c = move.Car()

dirs = os.listdir(save_path)
print(len(dirs))
# c.car_forward(210,210)
# exit(0)

count = 0
while count < 200:
    # 捕获一帧图像
    ret, frame = cap.read()

    # 如果捕获成功
    if ret:
        name = 'img_' + str(count+len(dirs)) + '.jpg'
        show_lcd(frame)
        # 给图片命名
        img_name = os.path.join(save_path, name)
        # print(img_name)

        # 保存图片
        cv2.imwrite(img_name, frame)

        print('Image %d saved.',count+len(dirs))

        # 图片计数器
        count += 1
    else:
        print('Failed to capture image.')

# 释放摄像头
cap.release()

print('All images are captured.')