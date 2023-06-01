import time

import cv2

import move
from YYJ import GPIO_RPi
from YYJ import Vision
from control import gpio
from main_program.util.mine_classify import MinesClassifier
from move import path

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


def PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax, SumMin, base_speed, break_mod=0, break_time=0, back_mod=0
                    , user_max_time=2):
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
    for i in range(1):  # Clear the buffer.
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
        sum = int((Sum[Line - 101] + Sum[Line - 102] + Sum[Line - 103] + \
                   Sum[Line - 104] + Sum[Line - 105] + Sum[Line - 106] + \
                   Sum[Line - 107] + Sum[Line - 108] + Sum[Line - 109]) / 3)  # 黑色像素点的数量 取9个点的平均值 原来是三个点的值
        if sum >= SumMax and break_flag > 40:  # 不要再刚转弯开始巡线就break
            max_time += 1
            # c.car_stop()
            # print("out max")
            # print(sum)
            # break
        elif max_time >= user_max_time:  # enough max time
            break
        # elif sum <= SumMin and one_path_done == 1 and break_mod == 0:
        #     one_path_done = 0
        #     c.car_stop()
        #     break
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
    # Cam.Release()
    # print('Tracking done')


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
    fork_road = [(11, 1), (1, 3), (13, 5), (15, 7), (3, 9), (17, 11), (5, 13), (7, 15), (19, 17), (9, 19)]
    # 1，1 19，5 15，7 1，15 5，13 19，19
    long_length_path = [(1, 1), (19, 5), (15, 7), (1, 15), (5, 13), (19, 19), (7, 7),
                        (13, 13)]  # it means those path need one more move step the other are the short length path
    for i in range(len(path)):  # 9 paths
        for j in range(len(path[i])):  # path points
            if path[i][j]["now_xy"] not in non_intersection_points:
                one_path.append(path[i][j]["move_mode"])
        if path[i][j]["now_xy"] in long_length_path and path[i][j]["move_mode"] == "前进":
            one_path.pop()
            one_path.append("长线")
        else:
            one_path.append("短线")
        move_list.append(one_path)
        one_path = []
    # print(move_list)
    for i in range(len(move_list) - 1):
        # print(i+1)
        # print(move_list[i+1][0])
        if move_list[i + 1][0] == "左转" and move_list[i + 1][1] == "左转":
            # no need the turning
            move_list[i + 1].pop(0)
            move_list[i + 1].pop(0)
        # change the direction
        elif move_list[i + 1][0] == "左转":
            move_list[i + 1][0] = "右转"
        elif move_list[i + 1][0] == "右转":
            move_list[i + 1][0] = "左转"
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
    # time.sleep(0.5)
    turn_right_90(0.43)
    # time.sleep(0.5)
    # c.car_stop()


def left_90():
    c.car_stop()
    # time.sleep(0.5)
    turn_left_90(0.43)
    # time.sleep(0.5)
    # c.car_stop()


def left_90_with_stop(turn_time=0.43):
    c.car_stop()
    time.sleep(0.5)
    turn_left_90(turn_time)
    time.sleep(0.5)
    c.car_stop()


def classify_treasure(team="red"):
    mine_dict = {
        0: "蓝色三角",
        1: "蓝色圆形",
        2: "红色圆形",
        3: "红色三角",
    }
    mine_classifier = MinesClassifier(paddle_model="./main_program/model/MobileNet_big.nb")  #
    for i in range(5):
        try:  # 多次重复调用有几率出错
            cap = cv2.VideoCapture(1)
            cap.set(4, 240)
            cap.set(3, 320)
            cam = "open"
            break
        except:
            cam = "cannot open"
            pass
    if cam == "cannot open":  # 五次都打不开直接return假宝藏
        print("Cannot open camera")
        class_of = "fake"
        return class_of
    list_of_treasure = [0, 0, 0, 0, 0]  # 0:蓝色三角 1:蓝色圆形 2:红色圆形 3:红色三角 4:无类别
    while True:
        success, img = cap.read()
        if success:
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
    # 分辨宝藏是真的还是假的,0:蓝色三角 1:蓝色圆形 2:红色圆形 3:红色三角
    if team == "red" and list_of_treasure[2] > 10:
        class_of = "fake"
    elif team == "red" and list_of_treasure[3] > 10:
        class_of = "true"
    elif team == "blue" and list_of_treasure[0] > 10:
        class_of = "fake"
    elif team == "blue" and list_of_treasure[1] > 10:
        class_of = "true"
    else:
        print("no treasure")
        class_of = "fake"
    return class_of


if __name__ == '__main__':
    move_list = slove_path(path)
    # below is the pid value
    K = 0.5
    Kp = 5
    Ki = 0
    Kd = 5
    Line = 120
    SumMax = 400
    SumMin = 100
    treasure_corner = 0
    one_path_done = 0  # in case for the stop on the midele of the path
    speed = 400
    break_time_long = 60  # 110
    break_time_short = 50
    c.car_forward(400, 400)
    time.sleep(1)
    c.car_stop()
    for i in range(len(move_list)):
        # i += 1
        print(i)
        for j in range(len(move_list[i])):
            if j == len(move_list[i]) - 1:
                one_path_done = 1
            if move_list[i][j] == "前进":
                treasure_corner = 0
                PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax, SumMin, speed)
                c.car_forward(400, 400)
                time.sleep(0.3)
                c.car_stop()
                # time.sleep(0.5)
            elif move_list[i][j] == "左转":
                treasure_corner = 1
                left_90()
                # print("turn left done")
                # time.sleep(0.5)
            elif move_list[i][j] == "右转":
                treasure_corner = 1
                right_90()
                # print("turn right done")
                # time.sleep(0.5)
        # finish one path
        if i == len(move_list) - 1:  # it mean it is the last path
            c.car_forward(400, 400)
            time.sleep(2)
            c.car_stop()
            break
        if move_list[i][j] == "长线":  #
            c.car_stop()
            time.sleep(0.3)
            # print("start long line")
            PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax + 5000, SumMin, 400, break_mod=1, break_time=break_time_long,
                            user_max_time=5)
            # print("long line done")
            # detect the treasure
            treasure = classify_treasure()
            print(treasure)
            if treasure == "fake":  # 掉头就跑
                left_90_with_stop()
                left_90_with_stop(0.50)
            elif treasure == "true":  # hit the treasure
                PIDLineTracking(K=0.5, Kp=5, Ki=0, Kd=3, Line=120, SumMax=450, SumMin=100, base_speed=350,
                                break_mod=1, break_time=break_time_short + 30)
                c.car_stop()
                time.sleep(0.5)
                hit_the_treasure(0.7)
                left_90_with_stop()
                left_90_with_stop(0.43)
        elif move_list[i][j] == "短线":  # done short line need to back to the cross road
            c.car_stop()
            time.sleep(0.3)
            # 观察宝藏
            treasure = classify_treasure()
            print(treasure)
            if treasure == "fake":
                if move_list[i + 1][0] == "前进":  # 需要掉头
                    left_90_with_stop()
                    left_90_with_stop(0.4)
                # 反转，因为没有撞宝藏 所以车头没有倒转
                elif move_list[i + 1][0] == "左转":
                    move_list[i + 1][0] = "右转"
                elif move_list[i + 1][0] == "右转":
                    move_list[i + 1][0] = "左转"
            elif treasure == "true":
                PIDLineTracking(K=0.5, Kp=5, Ki=0, Kd=3, Line=120, SumMax=450, SumMin=100, base_speed=350,
                                break_mod=1, break_time=break_time_short)
                # detect the treasure
                c.car_stop()
                time.sleep(0.5)
                hit_the_treasure(0.7)
                left_90_with_stop()
                left_90_with_stop(0.4)
                PIDLineTracking(K=0.5, Kp=5, Ki=0, Kd=2, Line=120, SumMax=450, SumMin=100, base_speed=350
                                , back_mod=0)  # back to the cross road

                c.car_forward(400, 400)
                time.sleep(0.3)
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
        print(treasure_corner, "one path done")

    # PIDLineTracking(K, Kp, Ki, Kd, Line, SumMax)
    print("test done")
