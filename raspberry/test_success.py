import move
import time
from control import gpio
from YYJ import Vision
from YYJ import GPIO_RPi
from move import path
import cv2
from util import LCD_2inch4

#init
c = move.Car()
i = move.Infrared()
S = gpio.Screen()
from PIL import Image, ImageFilter
disp = LCD_2inch4.LCD_2inch4()
disp.Init()
disp.clear()

def turn_left_90(time_move):
    c.car_turn_left(500, 500)
    time.sleep(time_move)
    c.car_stop()

def turn_right_90(time_move):
    c.car_turn_right(500, 500)
    time.sleep(time_move)
    c.car_stop()


def car_move_test():
    c.car_forward(1000,1000)
    time.sleep(5)
    # for i in range(6):
    #     turn_left_90(0.5)
    #     time.sleep(0.5)
    #     turn_right_90(0.5)
    #     time.sleep(0.5)
    c.car_back(1000,1000)
    time.sleep(5)
    c.car_stop()

def PIDLineTracking(K, Kp, Ki, Kd,Line,SumMax,SumMin):
    # 初始化摄像头
    car_speed = 280
    change_speed = 20
    global one_path_done
    sum = 0
    Cam = Vision.Camera(0, 320, 240)
    # 初始化PID模块
    PID = GPIO_RPi.PID(K, Kp, Ki, Kd, 160)
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
            c.car_stop()
            break
        if sum <= SumMin and one_path_done == 1:
            one_path_done = 0
            c.car_stop()
            break
        if pwm > 350:
            pwm = 350
        elif pwm < -350:
            pwm = -350
        c.car_forward(450-pwm,450+pwm)
        Cam.Delay(1)
    Cam.Release()
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
    for i in range (9): # 9 paths
        for j in range (len(path[i])):  # path points
            if path[i][j]["now_xy"] not in non_intersection_points:
                one_path.append(path[i][j]["move_mode"])
        move_list.append(one_path)
        one_path = []
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
    turn_right_90(0.4)
    time.sleep(0.5)
    c.car_stop()

def left_90():
    c.car_stop()
    time.sleep(0.5)
    turn_left_90(0.42)
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
    SumMax = 500
    SumMin = 100
    treasure_corner = 0
    one_path_done = 0  #in case for the stop on the midele of the path
    # hit_the_treasure(0.7)
    # print(move_list[0])
    # print(move_list[1])
    # exit()
    c.car_forward(400,400)
    time.sleep(1)
    c.car_stop()
    for i in range (len(move_list)):
        # i += 2
        print(i)
        for j in range (len(move_list[i])):
            if j == len(move_list[i])-1:
                one_path_done = 1
            if move_list[i][j] == "前进":
                treasure_corner = 0
                PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax,SumMin)
                c.car_forward(400,400)
                time.sleep(0.3)
                c.car_stop()
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
        if treasure_corner == 1:
            PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax,SumMin)
            c.car_forward(400, 400)
            time.sleep(0.3)
            c.car_stop()
            time.sleep(0.5)
            hit_the_treasure(0.5)
            left_90()
            left_90()
            PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax,SumMin)
            c.car_forward(400, 400)
            time.sleep(0.3)
            left_90()
            left_90()

        else:  #it mean it is the right treasure
            hit_the_treasure(0.5)
        print(treasure_corner,"one path done")

    # PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax)
    print("test done")

