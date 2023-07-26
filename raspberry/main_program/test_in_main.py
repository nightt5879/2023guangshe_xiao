# 包的导入
import sys

sys.path.insert(0, "/home/pi/.local/lib/python3.7/site-packages")
import os

os.chdir("/home/pi/Desktop/main_program")
from util.get_map import get_loc
from util.get_map import show_lcd
from util.get_path4 import pathPlaner
# from util.lidar3 import Lidar
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
BUTTON_INPUT = 18  # 按钮连接的GPIO口
SERVO_PIN_TOP = 17  # 云台舵机1连接的GPIO口
rotate_angle = 0  # 云台旋转的角度
TOP_ANGLE = 120  # 看宝藏的上面舵机角度
servo_top = 0  # 云台舵机1
callback_flag = 0  # 回调函数的标志位
hit_mine_flag = 0 # 反映是否撞了宝藏

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_INPUT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# 我也不知道为什么要在这里init input才行 用着用着就不行了 不管了 你就说能不能用把


cam = cv2.VideoCapture(-1)  # -1就是使用默认摄像头 防止报错
break_flag = 0
# 读五次图像，如果都成功就跳出
for i in range(5):
    success, img = cam.read()
    if success:
        show_lcd(img)
        break_flag += 1
        if (break_flag >= 5):
            print("摄像头来咯")
            break
    else:
        print("摄像头出不来哦")
        cam.release()  # 释放摄像头
        os.execl(sys.executable, sys.executable, *sys.argv)


def GPIO_init():
    """
    初始化所有的树莓派GPIO引脚
    """
    global rotate_angle, servo_top
    # 按键输入的init
    servo_top = servo.HalfCircleServo(SERVO_PIN_TOP)
    # # set the servo's rotate angle
    # servo_top.target = 90


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
        # if GPIO.input(BUTTON_INPUT) == GPIO.HIGH:
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


def create_new_process():
    global process, process_temp
    if process is not None and process_temp is None:
        process.terminate()
        # process_temp = Process(target=main_program)
        # process_temp.start()
        os.execl(sys.executable, sys.executable, *sys.argv)
    elif process is None and process_temp is not None:
        process_temp.terminate()
        # process = Process(target=main_program)
        # process.start()
        os.execl(sys.executable, sys.executable, *sys.argv)
def filp(move_input):
    """
    反转一下装了宝藏之后的移动
    Args:
        move_input: 输入的第一个移动指令
    Returns:
        反转之后的指令
    """
    if move_input == "左转":
        move_output = "右转"
    elif move_input == "右转":
        move_output = "左转"
    elif move_input == "掉头":
        move_output = "前进"
    elif move_input == "前进":
        move_output = "掉头"
    return move_output

# 回调函数，会在引脚状态改变时被调用
def callback_function(channel):
    global process
    print('Detected edge on channel %s' % channel)
    GPIO.remove_event_detect(BUTTON_INPUT)
    roll_back = button_with_wait()
    if (roll_back == "one_press"):
        print("不回退一个宝藏")
        # 保存不回退的宝藏
    elif (roll_back == "two_press"):
        print("回退一个宝藏")
        # 保存回退的宝藏
    with open("restart.txt", "w") as file:  # 创建"restart.txt"文件
        pass
    # 释放摄像头
    cam.release()
    # 重启程序
    # os.execl(sys.executable, sys.executable, *sys.argv)
    create_new_process()

if __name__ == '__main__':
    GPIO.remove_event_detect(BUTTON_INPUT)  # 关闭事件检测
    GPIO_init()  # 初始化GPIO
    # 启动舵机子线程
    t1 = threading.Thread(target=thread_nodding)
    t1.start()
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
        mine_points, mine_img = get_loc(cam)  # 摄像头捕获视频识别出宝藏位置
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
    # lidar = Lidar(img=map_array, model_path='./model/ultra_simple')  # 初始化雷达
    # MPU6050校准之类的工作
    # 准备出发
    img_start = cv2.imread("/home/pi/Desktop/main_program/util/imgs/ready_to_go.jpg")
    show_lcd(img_start)
    wait_the_press()  # 等待按键
    countdown(2)  # 倒计时
    img_start = cv2.imread("/home/pi/Desktop/main_program/util/imgs/go.jpg")
    show_lcd(img_start)
    control_servo(servo_top, 180, 0)
    GPIO.add_event_detect(BUTTON_INPUT, GPIO.FALLING, callback=callback_function, bouncetime=300)
    while True:
        path = planer.paths_list.pop(0)  # 删除并返回列表中的第一个元素
        for i in range (len(path)):  # 读取一个路径的移动指令
            move = path.pop(0)  # pop 出来指令
            if hit_mine_flag:  # 如果装了宝藏 肯定是需要掉头回去 这时候有一个指令相反的（车头朝向问题）
                hit_mine_flag = 0 # 清空撞了宝藏的标志位
                move = filp(move)
            if len(path) == 0: # 说明达到了最后一个移动指令 这个指令是朝向目标后还差几个格子
                print("到达目标点，距离宝藏距离为:", move)
                control_servo(servo_top, 90, 0)  # 抬起舵机
                # 识别宝藏
                mine = input("请输入宝藏情况：")# 对应关系--0：蓝色三角、1：蓝色圆形、2：红色圆形 、3：红色三角
                control_servo(servo_top, 180, 0) # 低下舵机
                # 如果是真的
                    # 撞
                # 如果不是真的
                    # 走
                print("之前的宝藏：", mine_points)
                planer.update(mine=0)  # 更新路径
                print(planer.ori_mines)
                mine_points = [((x + 1) // 2, 11 - (y + 1) // 2) for (x, y) in planer.ori_mines] # 转换成新的mine_points
                print("更新后的的宝藏：", mine_points)
                with open('mine_points.pkl', 'wb') as f:  # 保存新的宝藏txt文件
                    pickle.dump(mine_points, f)
                break  # 完成之后就不用再执行了 直接跳出循环 准备读下一个路径
            print("执行移动指令：", move) # 移动指令
            time.sleep(1) # 等待一秒 测试用的
        if len(planer.paths_list) == 0:
            break
        time.sleep(2)
    print("test done")
    # print(1)