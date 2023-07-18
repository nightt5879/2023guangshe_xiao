from util.get_map import get_loc
from util.get_map import show_lcd
from util.get_path import pathPlaner
from util.mine_classify import MinesClassifier
from util.get_map import button_input
# below is the other import
import cv2
import time
import RPi.GPIO as GPIO
import move
from YYJ import GPIO_RPi
from YYJ import Vision
import os
import sys
from move import path
from util.countdown import countdown

if __name__ == '__main__':
    team = "red"
    mine_points = get_loc()  # 摄像头捕获视频识别出宝藏位置
    countdown(10)  # 倒计时
    # print(1)
    # planer = pathPlaner(mine_points)  # 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长
