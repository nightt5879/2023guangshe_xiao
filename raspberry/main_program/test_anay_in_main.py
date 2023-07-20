import RPi.GPIO as GPIO
import time
# 包的导入
import sys
sys.path.insert(0, "/home/pi/.local/lib/python3.7/site-packages")
import os
os.chdir("/home/pi/Desktop/main_program")
from util.get_map import get_loc
from util.get_map import show_lcd
from util.get_path2 import pathPlaner
from util.lidar3 import Lidar
from util.mine_classify import MinesClassifier
from util.get_map import button_input
from util.map_rec import MapArchRecognizer

# below is the other import
import cv2
import time
import RPi.GPIO as GPIO
import move
from YYJ import GPIO_RPi
from YYJ import Vision
import sys
from move import path
from util.countdown import countdown
from util import servo
import threading
import pickle

def classify_treasure(team_of="red"):
    """
    识别宝藏
    :param team_of:  队伍颜色
    :return:
        class_of: 宝藏的类别
        color_of: 宝藏的颜色
    """
    mine_dict = {
        0: "蓝色三角",
        1: "蓝色圆形",
        2: "红色圆形",
        3: "红色三角",
    }
    mine_classifier = MinesClassifier(paddle_model="./model/MobileNetV2 7 13.nb")  # 加载模型
    color_of_treasure = ""  # 用于得到宝藏的颜色
    for i in range(5):
        try:  # 多次重复调用有几率出错
            cap = cv2.VideoCapture(-1)
            cap.set(4, 240)
            cap.set(3, 320)
            cam = "open"
            break
        except:
            cam = "cannot open"
            pass
    if cam == "cannot open":  # 五次都打不开直接return假宝藏
        print("Cannot open camera")
        class_of = "fake"
        return class_of
    list_of_treasure = [0, 0, 0, 0, 0]  # 0:蓝色三角 1:蓝色圆形 2:红色圆形 3:红色三角 4:无类别
    while True:
        success, img = cap.read()
        if success:
            cv2.imshow("img", img)
            result, pro = mine_classifier.recognize_img(img)
            print(result,pro)
            # print(pro)
            # if mine_dict[result] == "蓝色三角" and pro[0] > 0.42:
            #     print("蓝色三角")
            #     list_of_treasure[0] += 1
            # elif mine_dict[result] == "蓝色圆形" and pro[1] > 0.42:
            #     print("蓝色圆形")
            #     list_of_treasure[1] += 1
            # elif mine_dict[result] == "红色圆形" and pro[2] > 0.42:
            #     print("红色圆形")
            #     list_of_treasure[2] += 1
            # elif mine_dict[result] == "红色三角" and pro[3] > 0.42:
            #     print("红色三角")
            #     list_of_treasure[3] += 1
            # else:
            #     list_of_treasure[4] += 1
            # # sum of the list_of_treasure
            # if list_of_treasure[0] > 10 or list_of_treasure[1] > 10 or \
            #         list_of_treasure[2] > 10 or list_of_treasure[3] > 10 or list_of_treasure[4] > 10:
            #     break
            cv2.waitKey(1)
    print(list_of_treasure)
    max_index = list_of_treasure.index(max(list_of_treasure))  # 找到最大的下角标
    if max_index <= 3:  # 防止识别到的是no_treasure的情况
        color_of_treasure = mine_dict[max_index][:2]  # 取出前两个字得到颜色
    # 分辨宝藏是真的还是假的,0:蓝色三角 1:蓝色圆形 2:红色圆形 3:红色三角
    if team_of == "red" and list_of_treasure[2] > 10:
        class_of = "fake"
    elif team_of == "red" and list_of_treasure[3] > 10:
        class_of = "true"
    elif team_of == "blue" and list_of_treasure[0] > 10:
        class_of = "fake"
    elif team_of == "blue" and list_of_treasure[1] > 10:
        class_of = "true"
    else:
        print("no treasure")
        class_of = "fake"
    return class_of, color_of_treasure

if __name__ == '__main__':
    answer = classify_treasure()
    print(answer)
