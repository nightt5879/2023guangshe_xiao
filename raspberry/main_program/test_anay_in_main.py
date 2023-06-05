from util.get_path import pathPlaner
from util.mine_classify import MinesClassifier
# below is the other import
import cv2
import time


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
                               (3,1), (3,15), (17,5), (17,19),(7,9),(13,11)]
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
    one_path = []
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
    elif hit_flag == "no_hit":  # do not need to change the direction
        pass
    return move_list


def classify_treasure(team_of="red"):
    """
    识别宝藏
    :param team_of:  队伍颜色
    :return:
        class_of: 宝藏的类别
        color_of: 宝藏的颜色
    """
    mine_dict = {
        0: "蓝色三角",
        1: "蓝色圆形",
        2: "红色圆形",
        3: "红色三角",
    }
    mine_classifier = MinesClassifier(paddle_model="./model/MobileNet_big.nb")  # 加载模型
    color_of_treasure = ""  # 用于得到宝藏的颜色
    cam = ""
    global cap
    # for i in range(5):
    #     try:  # 多次重复调用有几率出错
    #         cap = cv2.VideoCapture(1)
    #         cap.set(4, 240)
    #         cap.set(3, 320)
    #         cam = "open"
    #         break
    #     except:
    #         print("打不开摄像头")
    #         cam = "cannot open"
    #         pass
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
    c.car_forward(400, 400)
    time.sleep(hit_treasure_time)
    c.car_stop()
    time.sleep(0.5)
    c.car_back(400, 400)
    time.sleep(hit_treasure_time)
    c.car_stop()
    time.sleep(0.5)

if __name__ == '__main__':
    mine_points = [(7, 10), (3, 10), (7, 6), (3, 6), (8, 5), (4, 5), (8, 1), (4, 1)]
    planer = pathPlaner(mine_points)  # 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长
    for i in range(len(planer.paths)):
        now_path = planer.paths.pop(0)  # 取总路径中第一个路径为当前要走的路径
        move_list = slove_path(now_path)  # 提取出需要的指令
        # print(move_list)



# from util.lidar import Lidar
# # 初始化激光雷达，用法是scan_data = my_lidar.get_data()。其中scan_data的值是一个列表，列表里面有360个元素分别对应0到359度范围的障碍物的距离
# my_lidar = Lidar()
# while True:
#     # 获取激光雷达数据
#     print(my_lidar.get_data())

