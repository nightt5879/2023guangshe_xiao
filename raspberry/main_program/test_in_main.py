# 包的导入
from util.get_map import get_loc
from util.get_map import show_lcd
from util.get_path2 import pathPlaner
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
# 全局参数
BUTTON_INPUT = 21  # 按钮连接的GPIO口

def GPIO_init():
    """
    初始化所有的树莓派GPIO引脚
    """
    # 按键输入的init
    GPIO.setmode(GPIO.BCM)
    BUTTON_PIN = BUTTON_INPUT
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def button_with_wait():
    """
    按下按钮后，等待一段时间，如果没有再次按下按钮，就返回one_press 此时对应的是红队
    如果再次按下按钮，就返回two_press 此时对应的是蓝队
    :return: one_press or two_press
    """
    press_flag = 0
    press_flag_down = 0
    press_time = 0
    while True:
        # 如果按键被按下
        # if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        #     print('Button is not pressed')
        if press_flag == 1 and GPIO.input(BUTTON_INPUT) == GPIO.HIGH:  # 按下松开后开始计时
            press_time += 1
        if GPIO.input(BUTTON_INPUT) == GPIO.LOW:
            time.sleep(0.1)  # 按键消除抖动
            if GPIO.input(BUTTON_INPUT) == GPIO.LOW:
                press_flag = 1
                if press_flag_down == 1:  # 第二次按下
                    print("two_press")
                    button = "two_press"
                    break
                # print('Button is pressed'
        if GPIO.input(BUTTON_INPUT) == GPIO.HIGH and press_flag == 1:
            time.sleep(0.1)
            if GPIO.input(BUTTON_INPUT) == GPIO.HIGH:
                press_flag_down = 1
        if press_time > 10:
            print("one_press")
            button = "one_press"
            break
        # print(press_time)
        # 暂停一段时间
        time.sleep(0.1)
    # GPIO.cleanup()  lcd需要使用不能清除GPIO口
    return button


def select_team():
    """
    选择队伍 按一下是红队 按两下是蓝队
    :return: red or blue
    """
    team_of = ""
    img_start = cv2.imread("/home/pi/Desktop/main_program/util/imgs/select.jpg")
    show_lcd(img_start)  # show the picture of the select team
    select_button = button_with_wait()
    if select_button == "one_press":
        img_start = cv2.imread("/home/pi/Desktop/main_program/util/imgs/red.jpg")
        show_lcd(img_start)  # show the picture of the select team
        team_of = "red"
    elif select_button == "two_press":
        team_of = "blue"
        img_start = cv2.imread("/home/pi/Desktop/main_program/util/imgs/blue.jpg")
        show_lcd(img_start)  # show the picture of the select team
    return team_of


if __name__ == '__main__':
    GPIO_init()  # 初始化GPIO
    team = select_team() # 选择队伍
    mine_points,mine_img = get_loc()  # 摄像头捕获视频识别出宝藏位置
    time.sleep(1)
    planer = pathPlaner(mine_points,team)  # 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长
    print(planer.paths_list)
    countdown(10)  # 倒计时
    # print(1)
    # planer = pathPlaner(mine_points)  # 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长
