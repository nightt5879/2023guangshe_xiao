from util.get_map import get_loc
from util.get_map import show_lcd
from util.get_path import pathPlaner
from util.lidar import Lidar
from util.mine_classify import MinesClassifier

import cv2
import time
import RPi.GPIO as GPIO


def button_with_wait() -> str:
    """
    按下按钮后，等待一段时间，如果没有再次按下按钮，就返回one_press 此时对应的是红队
    如果再次按下按钮，就返回two_press 此时对应的是蓝队
    :return: one_press or two_press
    """
    BUTTON_PIN = 18  # 按钮连接的GPIO口
    # 选择BCM模式
    GPIO.setmode(GPIO.BCM)
    # 设置GPIO口为输入
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    press_flag = 0
    press_flag_down = 0
    press_time = 0
    while True:
        # 如果按键被按下
        # if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        #     print('Button is not pressed')
        if press_flag == 1 and GPIO.input(BUTTON_PIN) == GPIO.HIGH:  # 按下松开后开始计时
            press_time += 1
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.1)  # 按键消除抖动
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                press_flag = 1
                if press_flag_down == 1:  # 第二次按下
                    print("two_press")
                    button = "two_press"
                    break
                # print('Button is pressed'
        if GPIO.input(BUTTON_PIN) == GPIO.HIGH and press_flag == 1:
            time.sleep(0.1)
            if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
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


def select_team() -> str:
    """
    选择队伍 按一下是红队 按两下是蓝队
    :return: red or blue
    """
    team_of = ""
    img_start = cv2.imread("/home/pi/Desktop/guangshe2023/main_program/util/select.jpg")
    show_lcd(img_start)  # show the picture of the select team
    select_button = button_with_wait()
    if select_button == "one_press":
        img_start = cv2.imread("/home/pi/Desktop/guangshe2023/main_program/util/red.jpg")
        show_lcd(img_start)  # show the picture of the select team
        team_of = "red"
    elif select_button == "two_press":
        team_of = "blue"
        img_start = cv2.imread("/home/pi/Desktop/guangshe2023/main_program/util/blue.jpg")
        show_lcd(img_start)  # show the picture of the select team
    return team_of


if __name__ == '__main__':
    # 初始化宝藏分类器，用法是：result = mine_classifier.recognize_img(img)。其中result的结果是0、1、2、3分别对应宝藏的四个类别。
    mine_classifier = MinesClassifier(paddle_model="./model/MobileNet_small.nb")
    team = select_team()  # 本次比赛的队伍颜色
    mine_points = get_loc()  # 摄像头捕获视频识别出宝藏位置
    planer = pathPlaner(mine_points)  # 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长
    print(mine_points)
    print(planer.paths)



# from util.lidar import Lidar
# # 初始化激光雷达，用法是scan_data = my_lidar.get_data()。其中scan_data的值是一个列表，列表里面有360个元素分别对应0到359度范围的障碍物的距离
# my_lidar = Lidar()
# while True:
#     # 获取激光雷达数据
#     print(my_lidar.get_data())

