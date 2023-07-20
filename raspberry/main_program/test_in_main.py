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

# 全局参数
BUTTON_INPUT = 21  # 按钮连接的GPIO口
SERVO_PIN_TOP = 24  # 云台舵机1连接的GPIO口
SERVO_PIN_MEDIUM = 23  # 云台舵机2连接的GPIO口
SERVO_PIN_BOTTOM = 18  # 云台舵机3连接的GPIO口
rotate_angle = 0  # 云台旋转的角度
TOP_ANGLE = 120 # 看宝藏的上面舵机角度


def GPIO_init():
    """
    初始化所有的树莓派GPIO引脚
    """
    global rotate_angle, servo_top, servo_medium, servo_bottom
    # 按键输入的init
    GPIO.setmode(GPIO.BCM)
    BUTTON_PIN = BUTTON_INPUT
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    servo_top = servo.HalfCircleServo(SERVO_PIN_TOP)
    servo_medium = servo.HalfCircleServo(SERVO_PIN_MEDIUM)
    servo_bottom = servo.HalfCircleServo(SERVO_PIN_BOTTOM)
    # set the servo's rotate angle
    servo_top.target = 90
    servo_medium.target = 0
    servo_bottom.target = 0


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
def wait_the_press():
    """
    等待按键按下
    """
    while True:
        if GPIO.input(BUTTON_INPUT) == GPIO.LOW:
            time.sleep(0.1)  # 按键消除抖动
            if GPIO.input(BUTTON_INPUT) == GPIO.LOW:
                    print('Button is pressed')
                    break


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


def control_servo(vertical_servo, vertical_angle: int, horizontal_angle: int):
    global rotate_angle
    vertical_servo.target = vertical_angle
    rotate_angle = horizontal_angle


def thread_nodding():
    while True:
        if servo_top.target != servo_top.angle:
            servo_top.set_angle(servo_top.target)
        time.sleep(0.5)


def thread_rotating():
    global rotate_angle
    while True:
        if rotate_angle != servo_medium.angle + servo_bottom.angle:
            rotate_angle = servo_medium.clamp_number(rotate_angle, 0, 360)
            if 0 <= rotate_angle <= 180:
                # check the servo_bottom's angle
                if servo_bottom.angle != 0:
                    servo_bottom.set_angle(0)
                servo_medium.set_angle(rotate_angle)
            elif 180 < rotate_angle <= 360:
                servo_medium.set_angle(180)
                # print(rotate_angle)
                servo_bottom.set_angle(rotate_angle - 180)
        time.sleep(0.5)

# 回调函数，会在引脚状态改变时被调用
def callback_function(channel):
    print('Detected edge on channel %s' % channel)
    with open("restart.txt", "w") as file:  # 创建"restart.txt"文件
        pass
    os.system("python3 /home/pi/Desktop/main_program/test_in_main.py")


if __name__ == '__main__':
    GPIO.remove_event_detect(BUTTON_INPUT) # 关闭事件检测
    GPIO_init()  # 初始化GPIO
    # 启动舵机子线程
    t1 = threading.Thread(target=thread_rotating)
    t2 = threading.Thread(target=thread_nodding)
    t1.start()
    t2.start()
    # 查看状态
    if os.path.exists("restart.txt"):
        print("中断开始")
        # 读取宝藏位置和队伍颜色
        with open('mine_points.pkl', 'rb') as f:
            mine_points = pickle.load(f)
        with open('team.pkl', 'rb') as f:
            team = pickle.load(f)
        mine_img = cv2.imread("mine_img.png")  # 读取"mine_img.png"文件
        # 读取完成后是一样的操作
        mapAnalysiser = MapArchRecognizer(mine_img)  # 实例化一个地图架构解析对象
        map_array = mapAnalysiser.analysis_map()  # 转换地图得到21 * 21的矩阵
        cv2.imwrite("./imgs/small_labyrinth.png", map_array)  # 把识别出来的21 * 21矩阵保存起来
        planer = pathPlaner(mine_points, team)  # 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长)
        os.remove("restart.txt")  # 删除"restart.txt"文件
        countdown(3)  # 倒计时
    else:
        print("正常的开始")
        team = select_team()  # 选择队伍
        control_servo(servo_top, 90, 90)
        mine_points, mine_img = get_loc()  # 摄像头捕获视频识别出宝藏位置
        mapAnalysiser = MapArchRecognizer(mine_img)  # 实例化一个地图架构解析对象
        # 下面是保存的部分
        cv2.imwrite("mine_img.png", mine_img)  # 把识别出来的地图保存起来
        with open('mine_points.pkl', 'wb') as f:
            pickle.dump(mine_points, f)
        with open('team.pkl', 'wb') as f:
            pickle.dump(team, f)
        # 下面是正常读取部分
        map_array = mapAnalysiser.analysis_map()  # 转换地图得到21 * 21的矩阵
        cv2.imwrite("./imgs/small_labyrinth.png", map_array)  # 把识别出来的21 * 21矩阵保存起来
        planer = pathPlaner(mine_points, team)  # 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长)
        countdown(2)  # 倒计时
    # time.sleep(1)
    print(planer.paths_list)
    print(mine_points)
    print(team)
    lidar = Lidar(img=map_array,model_path='./model/ultra_simple')  # 初始化雷达
    # MPU6050校准之类的工作
    # 准备出发
    img_start = cv2.imread("/home/pi/Desktop/main_program/util/imgs/ready_to_go.jpg")
    show_lcd(img_start)
    wait_the_press()  # 等待按键
    countdown(2)  # 倒计时
    img_start = cv2.imread("/home/pi/Desktop/main_program/util/imgs/go.jpg")
    show_lcd(img_start)
    GPIO.add_event_detect(BUTTON_INPUT, GPIO.FALLING, callback=callback_function, bouncetime=300)
    while planer.paths_list:
        path = planer.paths_list.pop(0)  # 删除并返回列表中的第一个元素
        action = path['action']
        direct = path['direct']
        print(direct)
        if direct == "上":
            control_servo(servo_top, TOP_ANGLE, 180)
        elif direct == "下":
            control_servo(servo_top, TOP_ANGLE, 0)
        elif direct == "左":
            control_servo(servo_top, TOP_ANGLE, 270)
        elif direct == "右":
            control_servo(servo_top, TOP_ANGLE, 90)
        while action:  # 执行一个路径
            action_move = action.pop(0)
            print(action_move)
            time.sleep(1)
        # 宝藏识别
        # if true  撞宝藏
        # elif fake 不撞
        # 更新路劲
        # 保存此时的宝藏位置
        planer.update_paths()
        print (mine_points)

        time.sleep(2)

    # print(1)
