from time import time
from util.get_map import get_loc
from util.get_path import pathPlaner
from util.lidar import Lidar
from util.mine_classify import MinesClassifier
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
    for i in range(9):  # 9 paths
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
    print(move_list)
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
# 摄像头捕获视频识别出宝藏位置
mine_points = get_loc()
# 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长
planer = pathPlaner(mine_points)
print(mine_points)
print(planer.paths)
# 初始化激光雷达，用法是scan_data = my_lidar.get_data()。其中scan_data的值是一个列表，列表里面有360个元素分别对应0到359度范围的障碍物的距离
my_lidar = Lidar()
# 初始化宝藏分类器，用法是：result = mine_classifier.recognize_img(img)。其中result的结果是0、1、2、3分别对应宝藏的四个类别。
mine_classifier = MinesClassifier(paddle_model="./model/MobileNet_small.nb")
# while True:
#     # 获取激光雷达数据
#     print(my_lidar.get_data())
