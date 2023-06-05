from util.get_map import get_loc
from util.get_map import show_lcd
from util.get_path import pathPlaner
from util.mine_classify import MinesClassifier
# below is the other import
import cv2
import time
import RPi.GPIO as GPIO
import move
from YYJ import GPIO_RPi
from YYJ import Vision
from control import gpio
from move import path


def button_with_wait():
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


def select_team():
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
        team_of = "红色"
    elif select_button == "two_press":
        team_of = "蓝色"
        img_start = cv2.imread("/home/pi/Desktop/guangshe2023/main_program/util/blue.jpg")
        show_lcd(img_start)  # show the picture of the select team
    return team_of


def slove_path(path, hit_flag="hit"):
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
                               (11, 11), (1, 13), (13, 15), (19, 15), (9, 17), (11, 19),
                               (17,5), (17,19), (3,1), (3,15),
                               (3,1), (3,15), (17,5), (17,19),(7,9),(13,11)]                   # 统一 所有的非交叉路口+直走都去掉
    # 11,1 1,3 13,5 15,7 3,9 17,11 5,13 7,15 19,17 9,19
    fork_road = [(11, 1), (1, 3), (13, 5), (15, 7), (3, 9), (17, 11), (5, 13), (7, 15), (19, 17), (9, 19)]
    # 1，1 19，5 15，7 1，15 5，13 19，19
    long_length_path = [(1, 1), (19, 5), (15, 7), (1, 15), (5, 13), (19, 19), (7, 7),
                        (13, 13)]  # it means those path need one more move step the other are the short length path
    for j in range(len(path)):  # path points
        if path[j]["now_xy"] in non_intersection_points and path[j]["move_mode"] == "前进":  # 只需要消除这个点上的前进指令
            pass
        else:
            one_path.append(path[j]["move_mode"])
    if path[j]["now_xy"] in long_length_path and path[j]["move_mode"] == "前进":
        one_path.pop()
        one_path.append("长线")
    else:
        one_path.append("短线")
    move_list.append(one_path)
    print("origin:",move_list)
    # if hit the treasure, the car turnover.so it need to change the direction
    if hit_flag == "hit":
        if move_list[0][0] == "左转" and move_list[0][1] == "左转":
            # no need the turning
            move_list[0].pop(0)
            move_list[0].pop(0)
        # change the direction
        elif move_list[0][0] == "左转":
            move_list[0][0] = "右转"
        elif move_list[0][0] == "右转":
            move_list[0][0] = "左转"
    elif hit_flag == "no_hit":  # do not need to change the direction
        pass
    print("new:", move_list)
    return move_list


def PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax, SumMin, base_speed, break_mod=0, break_time=0, back_mod=0,
                    user_max_time=2,non_break_time = 40):
    """
    PID巡线
    :param K: 总体的缩放比例
    :param Kp: 比例参数
    :param Ki: 积分参数
    :param Kd: 微分参数
    :param Line: 需要巡线的线位置
    :param SumMax: 黑色像素点max阈值
    :param SumMin: 黑色像素点你min阈值
    :param base_speed: 小车基本速度
    :param break_mod: 是否需要在规定时间内停止
    :param break_time: 规定的时间
    :param back_mod: 倒车（没用了）
    :param user_max_time: 达到max阈值的次数，满足后才会break。防止偶然误差
    :return: no return
    """
    # 初始化摄像头
    # global Cam
    global one_path_done
    break_flag = 0  # for the pid in time break
    max_time = 0
    # 初始化PID模块
    PID = GPIO_RPi.PID(K, Kp, Ki, Kd, 160)
    for i in range(1):  # Clear the buffer.
        Cam.ReadImg(0, 320, 0, 150)
        cv2.waitKey(1)
    while True:
        Cam.ReadImg(0, 320, 0, 150)
        Centre, Sum, Dst = Cam.LineTracking(Cam.Img, Line)
        Cam.ShowImg(Cam.Img)
        Cam.ShowImg(Dst, 'Dst')
        Now = int((Centre[Line] +
                   Centre[Line - 5] + Centre[Line + 5] +
                   Centre[Line - 4] + Centre[Line + 4] +
                   Centre[Line - 3] + Centre[Line + 3] +
                   Centre[Line - 2] + Centre[Line + 2] +
                   Centre[Line - 1] + Centre[Line + 1]) / 11)  # 十个附近点的值求平均
        Future = Centre[Line - 10]  # “未来”要去到的值
        D = Future - Now  # 差值
        PWM = PID.OneDin(Now)
        pwm = int(PWM)
        # print(pwm)
        sum = int((Sum[Line - 101] + Sum[Line - 102] + Sum[Line - 103] + Sum[Line - 104] + Sum[Line - 105] + Sum[Line - 106] +
                   Sum[Line - 107] + Sum[Line - 108] + Sum[Line - 109]) / 3)  # 黑色像素点的数量 取9个点的平均值 原来是三个点的值
        if sum >= SumMax and break_flag > non_break_time:  # 不要再刚转弯开始巡线就break
            max_time += 1
        elif max_time >= user_max_time:  # enough max time
            break
        if break_mod == 1 and break_flag >= break_time:
            print("break time max")
            c.car_stop()
            break
        pwm_1 = base_speed - pwm
        pwm_2 = base_speed + pwm
        # pwm range is 0-1000
        pwm_1 = max(0, min(1000, pwm_1))
        pwm_2 = max(0, min(1000, pwm_2))
        if back_mod == 0:
            c.car_forward(pwm_1, pwm_2)
        elif back_mod == 1:
            c.car_back(pwm_1, pwm_2)
        break_flag += 1
        Cam.Delay(1)
    c.car_stop()


def classify_treasure(team_of="red"):
    """
    识别宝藏
    :param team_of:  队伍颜色
    :return:
        class_of: 宝藏的类别
        color_of: 宝藏的颜色
    """
    global cap
    mine_dict = {
        0: "蓝色三角",
        1: "蓝色圆形",
        2: "红色圆形",
        3: "红色三角",
    }
    mine_classifier = MinesClassifier(paddle_model="./model/MobileNet_big.nb")  # 加载模型
    color_of_treasure = ""  # 用于得到宝藏的颜色
    list_of_treasure = [0, 0, 0, 0, 0]  # 0:蓝色三角 1:蓝色圆形 2:红色圆形 3:红色三角 4:无类别
    print("in here")
    while True:
        success, img = cap.read()
        if success:
            print("success")
            cv2.imshow("img", img)
            result, pro = mine_classifier.recognize_img(img)
            # print(pro)
            if mine_dict[result] == "蓝色三角" and pro[0] > 0.42:
                print("蓝色三角")
                list_of_treasure[0] += 1
            elif mine_dict[result] == "蓝色圆形" and pro[1] > 0.42:
                print("蓝色圆形")
                list_of_treasure[1] += 1
            elif mine_dict[result] == "红色圆形" and pro[2] > 0.42:
                print("红色圆形")
                list_of_treasure[2] += 1
            elif mine_dict[result] == "红色三角" and pro[3] > 0.42:
                print("红色三角")
                list_of_treasure[3] += 1
            else:
                list_of_treasure[4] += 1
            # sum of the list_of_treasure
            if list_of_treasure[0] > 10 or list_of_treasure[1] > 10 or \
                    list_of_treasure[2] > 10 or list_of_treasure[3] > 10 or list_of_treasure[4] > 10:
                break
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

def hit_the_treasure(hit_treasure_time):
    """
    撞击宝藏
    :param hit_treasure_time: 撞击运动的时间
    :return: no return
    """
    c.car_forward(450, 450)
    time.sleep(hit_treasure_time)
    c.car_stop()
    time.sleep(0.5)
    c.car_back(450, 450)
    time.sleep(hit_treasure_time)
    c.car_stop()
    time.sleep(0.5)

if __name__ == '__main__':
    # team = select_team()  # 本次比赛的队伍颜色
    # mine_points = get_loc()  # 摄像头捕获视频识别出宝藏位置
    mine_points = [(7, 10), (3, 10), (7, 6), (3, 6), (8, 5), (4, 5), (8, 1), (4, 1)]
    planer = pathPlaner(mine_points)  # 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长
    team = "红色"
    c = move.Car()
    i = move.Infrared()
    S = gpio.Screen()
    Cam = Vision.Camera(0, 320, 240)
    cap = cv2.VideoCapture(1)
    cap.set(4, 240)
    cap.set(3, 320)
    # init all the variable
    hit_flag = "no_hit"
    # K is the proportion.the other are the parameter of the PID
    K = 0.5
    Kp = 5
    Ki = 0
    Kd = 5
    Line = 120  # which line choose to follow
    SumMax = 400  # max of the black points
    SumMin = 100  # min of the black points
    treasure_corner = 0
    one_path_done = 0  # in case for the stop in the middle of the path
    speed = 400  # the car speed
    break_time_long = 80  # 110  long line break time
    break_time_short = 50  # short line break time
    # below is the start of the program
    c.car_forward(400, 400)    # go forward to get in the maze
    time.sleep(1)
    c.car_stop()
    #开头先处理一个path
    now_path = planer.paths.pop(0)  # 取总路径中第一个路径为当前要走的路径
    move_list = slove_path(now_path, hit_flag)  # 提取出需要的指令
    # below is the main loop
    while True:
        for j in range(len(move_list[0])):
            if j == len(move_list[0]) - 1:
                one_path_done = 1
            if move_list[0][j] == "前进":
                treasure_corner = 0
                PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax, SumMin, speed)
                c.car_forward(400, 400)
                time.sleep(0.3)
                c.car_stop()
                # print("forward done")
            elif move_list[0][j] == "左转":
                treasure_corner = 1
                c.car_turn_left_6050(5000)
                # print("left done")
            elif move_list[0][j] == "右转":
                treasure_corner = 1
                c.car_turn_right_6050(5000)
                # print("right done")
        # finish one path
        if move_list[0][j] == "长线":  #
            c.car_stop()
            time.sleep(1)
            # print("start long line")
            PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax + 5000, SumMin, 400, break_mod=1, break_time=break_time_long,
                            user_max_time=5)
            # detect the treasure
            treasure,color = classify_treasure()
            if color == team and len(planer.paths) != 0:  # 颜色相同并且
                print("优化路径")
                planer.update_paths()
            print(treasure,color)
            if treasure == "fake":  # 掉头就跑
                hit_flag = "hit" # 长线一律当撞过 掉头
                c.car_turn_right_6050(12000)
                PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax, SumMin, 400,non_break_time=10)  # 回到开头交叉路口
                c.car_forward(400, 400)
                time.sleep(0.3)
                c.car_stop()
                print("long no hit done")
            elif treasure == "true":  # hit the treasure
                hit_flag = "hit" # 长线一律当撞过 掉头
                PIDLineTracking(K=0.5, Kp=5, Ki=0, Kd=3, Line=120, SumMax=450, SumMin=100, base_speed=350,
                                break_mod=1, break_time=break_time_short)
                c.car_stop()
                time.sleep(0.5)
                hit_the_treasure(0.7)
                c.car_turn_right_6050(12000)
                PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax, SumMin, 400,
                                non_break_time=30)  # 回到开头交叉路口
                c.car_forward(400, 400)
                time.sleep(0.3)
                c.car_stop()
                print("long hit done")
        elif move_list[0][j] == "短线":  # done short line need to back to the cross road
            c.car_stop()
            time.sleep(1)
            # 观察宝藏
            treasure,color = classify_treasure()
            if color == team:  # 颜色相同，优化路径
                print("优化路径")
                planer.update_paths()
            print(treasure,color)
            if treasure == "fake":
                hit_flag = "no_hit"  # 没撞宝藏 等待下次更新
            elif treasure == "true":
                hit_flag = "hit"  # 撞了宝藏
                PIDLineTracking(K=0.5, Kp=5, Ki=0, Kd=3, Line=120, SumMax=450, SumMin=100, base_speed=400,
                                break_mod=1, break_time=break_time_short)
                # detect the treasure
                c.car_stop()
                time.sleep(0.5)
                hit_the_treasure(0.7)
                c.car_turn_right_6050(12000)
                PIDLineTracking(K=0.5, Kp=5, Ki=0, Kd=2, Line=120, SumMax=450, SumMin=100, base_speed=350
                                , back_mod=0)  # back to the cross road
                c.car_forward(400, 400)
                time.sleep(0.3)
                c.car_stop() # 等待下一次指令
        if len(planer.paths) == 0:  # 如果路径列表为空,表示已经找完了所有宝藏了,那么就要离开迷宫了
            c.car_forward(400, 400)
            time.sleep(2)  # 让小车继续直走一段距离然后就走出迷宫了
            c.car_stop()
            break
        now_path = planer.paths.pop(0)  # 取总路径中第一个路径为当前要走的路径
        move_list = slove_path(now_path, hit_flag)  # 提取出需要的指令

    print("test done")
    Cam.Release()
    cap.release()
