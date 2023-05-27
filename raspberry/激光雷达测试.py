from os import popen
from time import time
result = popen(" ./model/ultra_simple --channel --serial /dev/ttyAMA0 115200")
# 先把开头介绍激光雷达型号参数的那几行读出来避免影响后续数据处理
for i in range(6):
    result.buffer.readline().decode("utf8")
# 定义一个列表来装激光雷达返回的距离数据，0到359度
scan_data = [0] * 360
# 获取上一次循环得到的角度
last_angle = 0
# 获取上一次循环得到的距离
last_distance = 1
# 获取theta字符串所在位置的索引
index_theta = 10
start_list = []
isStart = False
# start = time()
while True:
    # 读取一行数据，数据类似："   theta: 6.80 Dist: 00133.00 Q: 47"
    line = result.buffer.readline().decode("utf8")
    # 获取Dist字符串所在位置索引
    index_Dist = line.find("Dist")
    # 获取Q字符串所在位置索引
    index_Q = line.rfind("Q")
    # 使用刚刚得到的索引来获取角度并转成int类型，而且角度在0到359范围内
    angle = min([359,int(float(line[index_theta:index_Dist-1]))])
    # 根据刚刚得到的索引来获取u建立并转成int类型
    distance = int(line[index_Dist+6:index_Q-4])
    # 把刚刚获取的距离根据角度放入列表的对应位置
    scan_data[angle] = distance if distance != 0 else last_distance
    # 如果角度为359并且上一个角度不是359的话，表示激光雷达已经扫描一圈d
    if angle == 359 and last_angle != 359:
        if isStart is False:
            # 90是右转，-90是左转
            start_list = scan_data[90:]+scan_data[:90]
            isStart = True
        else:
            dis = sum(([abs(start_list[i]-scan_data[i]) for i in range(len(start_list))]))
            print(dis)
        # print(time()-start)
        # start = time()
        # print(scan_data)
    last_angle = angle
    last_distance = distance
