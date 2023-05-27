from time import time
from util.get_map import get_loc
from util.get_path import get_paths
from util.lidar import Lidar
from util.mine_classify import MinesClassifier
# 摄像头捕获视频识别出宝藏位置
mine_points = get_loc()
# 根据宝藏位置得到最终的总运动指令,optimize=True的话。最终路径就是真正最短的，但是用时可能更长
path = get_paths(mine_points, optimize=False)
print(path)
# 初始化激光雷达，用法是scan_data = my_lidar.get_data()。其中scan_data的值是一个列表，列表里面有360个元素分别对应0到359度范围的障碍物的距离
my_lidar = Lidar()
# 初始化宝藏分类器，用法是：result = mine_classifier.recognize_img(img)。其中result的结果是0、1、2、3分别对应宝藏的四个类别。
mine_classifier = MinesClassifier(paddle_model="./model/MobileNet_small.nb")
while True:
    # 获取激光雷达数据
    print(my_lidar.get_data())