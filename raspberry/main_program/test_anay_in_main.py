import move
import time
import RPi.GPIO as GPIO
from YYJ import GPIO_RPi
from YYJ import Vision
Cam = Vision.Camera(0, 320, 240)
import cv2

def PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax, SumMin, base_speed, break_mod=0, break_time=0, back_mod=0,
                    user_max_time=2,set_break_flag=40):
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
        # Cam.ShowImg(Cam.Img)
        image = Dst
        # image = image.filter(ImageFilter.SHARPEN)
        # disp.ShowImage(image)
        # print(Centre[Line],sum)
        Now = int((Centre[Line] +
                   Centre[Line - 5] + Centre[Line + 5] +
                   Centre[Line - 4] + Centre[Line + 4] +
                   Centre[Line - 3] + Centre[Line + 3] +
                   Centre[Line - 2] + Centre[Line + 2] +
                   Centre[Line - 1] + Centre[Line + 1]) / 11)  # 十个附近点的值求平均
        # D = Future - Now  # 差值
        PWM = PID.OneDin(Now)
        pwm = int(PWM)
        # # print(pwm)
        sum = int(
            (Sum[Line - 1] + Sum[Line - 2] + Sum[Line - 3] + Sum[Line - 4] + Sum[Line - 5] + Sum[Line - 6] +
             Sum[Line - 7] + Sum[Line - 8] + Sum[Line - 9]) / 3)  # 黑色像素点的数量 取9个点的平均值 原来是三个点的值
        # print(sum,Now,pwm,max_time)
        if sum >= SumMax and break_flag > set_break_flag:  # 不要再刚转弯开始巡线就break
            max_time += 1
            # c.car_stop()
            print("out max")
            # print(sum)
            # break
        if max_time >= user_max_time:  # enough max time
            break
        # if break_mod == 1 and break_flag >= break_time:
        #     print("break time max")
        #     c.car_stop()
        #     break
        pwm_1 = base_speed - pwm
        pwm_2 = base_speed + pwm
        # pwm range is 0-1000
        pwm_1 = max(0, min(1000, pwm_1))
        pwm_2 = max(0, min(1000, pwm_2))
        c.car_forward(pwm_1, pwm_2)
        break_flag += 1
        Cam.Delay(1)
    c.car_stop()

def go_forward():
    PIDLineTracking(K=1, Kp=3, Ki=0, Kd=2, Line=100, SumMax=500, SumMin=100, base_speed=1000, break_mod=1,
                    set_break_flag=10, user_max_time=1)
    c.car_forward(1000, 1000)
    time.sleep(0.1)
    c.car_stop()

def turn_right():
    c.car_turn_right_6050(2300, 1000)
    c.car_cheak_data()
    c.car_stop()

def turn_left():
    c.car_turn_left_6050(2300, 1000)
    c.car_cheak_data()
    c.car_stop()
c = move.Car()
go_forward()
turn_right()
go_forward()
turn_left()
go_forward()
turn_right()
go_forward()
turn_right()
go_forward()
turn_right()
time.sleep(1)
turn_right()
go_forward()
turn_right()
go_forward()
turn_right()
go_forward()
go_forward()
turn_left()
go_forward()
go_forward()
go_forward()
turn_left()
time.sleep(1)
turn_left()
go_forward()
turn_right()
go_forward()

# c.car_forward(400,400)
# time.sleep(1)
# c.car_back(400,400)
# time.sleep(1)

print("Test done")





# DIS_X = 80
# DIS_Y = 80
# 下面是麦轮的测试 有缘再见
# def move_forward(distance):
#     c.car_send_distance_positive(distance, 0)
#     c.car_cheak_data()
#
# def move_backward(distance):
#     c.car_send_distance_negative(distance, 0)
#     c.car_cheak_data()
#
# def move_left(distance):
#     c.car_send_distance_negative(0, distance)
#     c.car_cheak_data()
#
# def move_right(distance):
#     c.car_send_distance_positive(0, distance)
#     c.car_cheak_data()
# if __name__ == "__main__":
#     # c.control_6050(0)
#     # time.sleep(1)
#     # c.control_6050(1)
#     move_left(DIS_X)
#     time.sleep(0.5)
#     move_forward(DIS_Y)
#     time.sleep(0.5)
#     move_left(DIS_X)
#     time.sleep(0.5)
#     move_forward(DIS_Y)
#     # time.sleep(0.5)
#     print("test done")
