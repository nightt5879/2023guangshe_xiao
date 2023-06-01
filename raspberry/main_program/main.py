from util.get_map import get_loc
from util.get_map import show_lcd
from util.get_path import pathPlaner
from util.mine_classify import MinesClassifier
# below is the other import
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


def slove_path(path, hit_flag="hit") -> list:
    """
    根据路径规划的结果，进行路径的解决,每次只解决一条路径（使用pop取出 ）
    :param path: 输入的路径
    :param hit_flag: 是否撞击了宝藏
    :return: 单条路径的list
    """
    move_list = []
    one_path = []
    # 9,1 11,3 1,5 7,5 19,7 9,9 11,9 15,9 5,11 9,11 11,11 1,13 13,15 19,15 9,17 11,19
    non_intersection_points = [(9, 1), (11, 3), (1, 5), (7, 5), (19, 7), (9, 9), (11, 9), (15, 9), (5, 11), (9, 11),
                               (11, 11), (1, 13), (13, 15), (19, 15), (9, 17), (11, 19)]
    # 11,1 1,3 13,5 15,7 3,9 17,11 5,13 7,15 19,17 9,19
    fork_road = [(11, 1), (1, 3), (13, 5), (15, 7), (3, 9), (17, 11), (5, 13), (7, 15), (19, 17), (9, 19)]
    # 1，1 19，5 15，7 1，15 5，13 19，19
    long_length_path = [(1, 1), (19, 5), (15, 7), (1, 15), (5, 13), (19, 19), (7, 7),
                        (13, 13)]  # it means those path need one more move step the other are the short length path
    for j in range(len(path)):  # path points
        if path[j]["now_xy"] not in non_intersection_points:
            one_path.append(path[j]["move_mode"])
    if path[j]["now_xy"] in long_length_path and path[j]["move_mode"] == "前进":
        one_path.pop()
        one_path.append("长线")
    else:
        one_path.append("短线")
    move_list.append(one_path)
    one_path = []
    print(move_list)
    # if hit the treasure, the car turnover.so it need to change the direction
    if hit_flag == "hit":
        # print(i+1)
        # print(move_list[i+1][0])
        if move_list[0][0] == "左转" and move_list[0][1] == "左转":
            # no need the turning
            move_list[0].pop(0)
            move_list[0].pop(0)
        # change the direction
        elif move_list[0][0] == "左转":
            move_list[0][0] = "右转"
        elif move_list[0][0] == "右转":
            move_list[0][0] = "左转"
    elif hit_flag == "no_hit":
        pass
    return move_list


if __name__ == '__main__':
    # 初始化宝藏分类器，用法是：result = mine_classifier.recognize_img(img)。其中result的结果是0、1、2、3分别对应宝藏的四个类别。
    mine_classifier = MinesClassifier(paddle_model="./model/MobileNet_big.nb")
    team = select_team()  # 本次比赛的队伍颜色
    mine_points = get_loc()  # 摄像头捕获视频识别出宝藏位置
    planer = pathPlaner(mine_points)  # 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长
    print(mine_points)
    print(planer.paths)

