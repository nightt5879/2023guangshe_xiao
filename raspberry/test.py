import cv2

import move
import time
from control import gpio
from YYJ import Vision
from YYJ import GPIO_RPi
from move import path
from util import LCD_2inch4

# init all the modules
c = move.Car()
i = move.Infrared()
S = gpio.Screen()
Cam = Vision.Camera(0, 320, 240)

def turn_left_90(time_move):
    c.car_turn_left(500, 500)
    time.sleep(time_move)
    c.car_stop()

def turn_right_90(time_move):
    c.car_turn_right(500, 500)
    time.sleep(time_move)
    c.car_stop()

def PIDLineTracking(K, Kp, Ki, Kd,Line,SumMax,SumMin,base_speed,break_mod =0,break_time = 0,back_mod = 0):
    # 初始化摄像头
    # global Cam
    global one_path_done
    sum = 0

    pwm_1 = 0
    pwm_2 = 0
    break_flag = 0  # for the pid in time break
    max_time = 0
    # 初始化PID模块
    PID = GPIO_RPi.PID(K, Kp, Ki, Kd, 160)
    for i in range (5):  # Clear the buffer.
        Cam.ReadImg(0, 320, 0, 150)
        cv2.waitKey(1)
    while True:
        Cam.ReadImg(0, 320, 0, 150)
        Centre, Sum, Dst = Cam.LineTracking(Cam.Img, Line)
        Cam.ShowImg(Cam.Img)
        Cam.ShowImg(Dst, 'Dst')
        image = Dst
        # image = image.filter(ImageFilter.SHARPEN)
        # disp.ShowImage(image)
        # print(Centre[Line],sum)
        Now = int((Centre[Line] + Centre[Line - 10])/2) # 现在的值
        Future = Centre[Line - 10]  # “未来”要去到的值
        D = Future - Now  # 差值
        PWM = PID.OneDin(Now)
        pwm = int(PWM)
        # print(pwm)
        sum = Sum[Line - 50] + Sum[Line - 40] + Sum[Line - 45]  # 黑色像素点的数量
        if sum >= SumMax:
            max_time += 1
            # c.car_stop()
            # print("out max")
            # print(sum)
            # break
        elif max_time >= 2:  # enough max time
            break
        elif sum <= SumMin and one_path_done == 1:
            one_path_done = 0
            c.car_stop()
            break
        if break_mod == 1 and break_flag >= break_time:
            c.car_stop()
            break
        pwm_1 = base_speed - pwm
        pwm_2 = base_speed + pwm
        # pwm range is 0-1000
        pwm_1 = max(0, min(1000, pwm_1))
        pwm_2 = max(0, min(1000, pwm_2))
        if back_mod == 0:
            c.car_forward(pwm_1,pwm_2)
        elif back_mod == 1:
            c.car_back(pwm_1,pwm_2)
        break_flag += 1
        Cam.Delay(1)
    # Cam.Release()
    print('Tracking done')

def slove_path(path):
    """
    Args:
        :param path: input standard path data.
    :return:
        move_list: output move list.
    """
    move_list = []
    one_path = []
    # 9,1 11,3 1,5 7,5 19,7 9,9 11,9 15,9 5,11 9,11 11,11 1,13 13,15 19,15 9,17 11,19
    non_intersection_points = [(9, 1), (11, 3), (1, 5), (7, 5), (19, 7), (9, 9), (11, 9), (15, 9), (5, 11), (9, 11),
                               (11, 11), (1, 13), (13, 15), (19, 15), (9, 17), (11, 19)]
    # 11,1 1,3 13,5 15,7 3,9 17,11 5,13 7,15 19,17 9,19
    fork_road = [(11,1), (1,3), (13,5), (15,7), (3,9), (17,11), (5,13), (7,15), (19,17), (9,19)]
    #1，1 19，5 15，7 1，15 5，13 19，19
    long_length_path = [(1,1), (19,5), (15,7), (1,15), (5,13), (19,19),(7,7),(13,13)]  # it means those path need one more move step the other are the short length path
    for i in range (9): # 9 paths
        for j in range (len(path[i])):  # path points
            if path[i][j]["now_xy"] not in non_intersection_points:
                one_path.append(path[i][j]["move_mode"])
        if path[i][j]["now_xy"] in long_length_path and path[i][j]["move_mode"] == "前进":
            one_path.pop()
            one_path.append("长线")
        else:
            one_path.append("短线")
        move_list.append(one_path)
        one_path = []
        # below is change the turning car way
        for i in range(len(move_list) - 1):
            if move_list[i+1][0] == "左转" and move_list[i+1][1]  =="左转":
                #no need the turning
                move_list[i+1].pop(0)
                move_list[i+1].pop(0)
            # change the direction
            elif move_list[i+1][0] == "左转":
                move_list[i+1][0] = "右转"
            elif move_list[i+1][0] == "右转转":
                move_list[i+1][0] = "左转"
    return move_list

def hit_the_treasure(hit_treasure_time):
    c.car_forward(400, 400)
    time.sleep(hit_treasure_time)
    c.car_stop()
    time.sleep(0.5)
    c.car_back(400, 400)
    time.sleep(hit_treasure_time)
    c.car_stop()
    time.sleep(0.5)

def right_90():
    c.car_stop()
    time.sleep(0.5)
    turn_right_90(0.45)
    time.sleep(0.5)
    c.car_stop()

def left_90():
    c.car_stop()
    time.sleep(0.5)
    turn_left_90(0.45)
    time.sleep(0.5)
    c.car_stop()

if __name__ == '__main__':
    move_list = slove_path(path)
    #below is the pid value
    K = 0.5
    Kp = 2
    Ki = 0
    Kd = 3
    Line = 60
    SumMax = 450
    SumMin = 100
    treasure_corner = 0
    one_path_done = 0  #in case for the stop on the midele of the path
    speed = 400
    break_time_long = 100
    break_time_short = 40
    # hit_the_treasure(0.7)
    # print(move_list[0])
    # print(move_list[1])
    # PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax,SumMin,speed)
    # c.car_forward(400,400)
    # time.sleep(0.3)
    # turn_right_90(0.4)
    # time.sleep(0.2)
    # PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax, SumMin, 400, break_mod=1, break_time=break_time_)
    # exit()
    c.car_forward(400,400)
    time.sleep(1)
    c.car_stop()
    for i in range (len(move_list)):
        i += 6
        print(i)
        for j in range (len(move_list[i])):
            if j == len(move_list[i])-1:
                one_path_done = 1
            if move_list[i][j] == "前进":
                treasure_corner = 0
                PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax,SumMin,speed)
                c.car_forward(400,400)
                time.sleep(0.3)
                c.car_stop()
                time.sleep(0.5)
            elif move_list[i][j] == "左转":
                treasure_corner = 1
                left_90()
                print("turn left done")
                time.sleep(0.5)
            elif move_list[i][j] == "右转":
                treasure_corner = 1
                right_90()
                print("turn right done")
                time.sleep(0.5)
        #finish one path
        if i == len(move_list)-1: #it mean it is the last path
            c.car_forward(400,400)
            time.sleep(2)
            c.car_stop()
            break
        if move_list[i][j] == "长线":  #
            PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax,SumMin,400,break_mod=1,break_time=break_time_long)
            # detect the treasure
            hit_the_treasure(0.7)
            left_90()
            left_90()
        elif move_list[i][j] == "短线": # done short line need to back to the cross road
            PIDLineTracking(K = 0.5, Kp = 2, Ki = 0, Kd = 3, Line = 60, SumMax = 450,SumMin = 100 ,base_speed = 400,
                            break_mod=1,break_time=break_time_short)
            # detect the treasure
            c.car_stop()
            time.sleep(0.5)
            hit_the_treasure(0.7)
            left_90()
            left_90()
            PIDLineTracking(K = 0.5, Kp = 3, Ki = 0, Kd = 0, Line = 60, SumMax = 450,SumMin = 100 ,base_speed = 300
                            , back_mod = 0)  # back to the cross road

            c.car_forward(400, 400)
            time.sleep(0.5)
            # left_90()
            # left_90()
            c.car_stop()

        # now the car in fornt of the treasure, we need to detect the treasure
        # detect code
        # if yse
        # hit_the_treasure(0.7)
        # else
        # c.car.stop()
        # if 1 == 1:
        #     hit_the_treasure(0.7)
        print(treasure_corner,"one path done")

    # PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax)
    print("test done")

