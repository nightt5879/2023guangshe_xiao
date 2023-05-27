import cv2
import numpy as np
import math
from time import time
import matplotlib.pyplot as plt
import itertools
import json
from random_map import Treasure

# 存储小车所有走过的路径,用于可视化路径规划
total_point = []


def get_obs(small_labyrinth_file="./img/small_labyrinth.png"):
    """
    读取读取压缩后的迷宫图片,获取迷宫墙壁。
    用于后续路径规划
    :param small_labyrinth_file: 迷宫图片文件所在位置
    :return:
    """
    # 读取图片
    img = cv2.imread(small_labyrinth_file, 0)
    # 获取黑色像素即墙壁的索引位置
    zero = np.where(img == 0)
    # 生成墙壁坐标点, 由于numpy矩阵起点是左上角, matplotlib画图的起点是左下角, 所以y方向上要转换坐标
    obs = set((ys, 20 - xs) for xs, ys in zip(zero[0], zero[1]))
    return obs


obs = get_obs()


class Env:
    def __init__(self):
        self.x_range = 21
        self.y_range = 21
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = obs

    def update_obs(self, obs):
        self.obs = obs


class ADStar:
    def __init__(self, s_start, s_goal, car_direct, plot, just_get_distance=False, eps=2.5, heuristic_type="euclidean"):
        """
        Anytime D*路径规划算法
        :param s_start: 起点坐标, 一般情况类似(1, 3)但是有时候会出现类似[(1, 3), "上"], 最后一个是用于让小车到达宝藏位置后朝向宝藏, 对于起点坐标来说没用
        :param s_goal:  终点坐标, 同上, 但是朝向是有用的, 会存储在self.target_direct中
        :param car_direct: 小车初始朝向, 上、下、左、右
        :param plot: 是否绘图, 在调试的时候推荐绘制, 可以可视化路径规划, 实际的时候为了提高运行速度, 不绘制
        :param just_get_distance: 是否是仅获取两个坐标点之间的距离而不获取具体运动指令, 一般在旅行商问题暴力枚举算法计算宝藏之间距离的时候设置为True
        :param eps: 路径规划的误差阈值, 当达到这个误差范围内的时候视为达到目标, 即可完成路径规划, 一般设置为2.5就可以了不需要变动
        :param heuristic_type: 同上, 不需要变动
        """
        # 下面有些变量在上面注释的时候已经解释了。解释过的就不会再进行注释
        self.s_start = s_start[0] if isinstance(s_start[-1], str) else s_start
        self.s_goal = s_goal[0] if isinstance(s_goal[-1], str) else s_goal
        self.car_direct = car_direct
        self.target_direct = s_goal[1]
        self.heuristic_type = heuristic_type
        self.plot = plot
        self.just_get_distance = just_get_distance

        # 下面的内容就是初始化迷宫地图以及路径规划参数和变量了, 都不需要变动
        self.Env = Env()
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.g, self.rhs, self.OPEN = {}, {}, {}

        for i in range(1, self.Env.x_range - 1):
            for j in range(1, self.Env.y_range - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.s_goal] = 0.0
        self.eps = eps
        self.OPEN[self.s_goal] = self.Key(self.s_goal)
        self.CLOSED, self.INCONS = set(), dict()

        self.title = "Anytime D*: Small changes"  # Significant changes

    def run(self):
        if self.plot:
            Plotting().plot_grid(self.title)

        self.ComputeOrImprovePath()
        path, total_distance, car_direct = self.get_path(
            [(x, y) for (x, y) in self.extract_path() if x % 2 == 1 and y % 2 == 1])

        return path, total_distance, car_direct

    def ComputeOrImprovePath(self):
        while True:
            s, v = self.TopKey()
            if v >= self.Key(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:
                break
            self.OPEN.pop(s)
            if self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                self.CLOSED.add(s)
                for sn in self.get_neighbor(s):
                    self.UpdateState(sn)
            else:
                self.g[s] = float("inf")
                for sn in self.get_neighbor(s):
                    self.UpdateState(sn)
                self.UpdateState(s)

    def UpdateState(self, s):
        if s != self.s_goal:
            self.rhs[s] = float("inf")
            for x in self.get_neighbor(s):
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
        if s in self.OPEN:
            self.OPEN.pop(s)

        if self.g[s] != self.rhs[s]:
            if s not in self.CLOSED:
                self.OPEN[s] = self.Key(s)
            else:
                self.INCONS[s] = 0

    def Key(self, s):
        if self.g[s] > self.rhs[s]:
            return [self.rhs[s] + self.eps * self.h(self.s_start, s), self.rhs[s]]
        else:
            return [self.g[s] + self.h(self.s_start, s), self.g[s]]

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.OPEN, key=self.OPEN.get)
        return s, self.OPEN[s]

    def h(self, s_start, s_goal):
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_start]
        s = self.s_start

        for k in range(200):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_goal:
                break

        return list(path)

    def get_path(self, path):
        """
        根据计算出来的路径, 生成运动指令, 路径长度, 小车最终朝向, 如果只需要距离的话, 运动指令和最终朝向都是None
        :param path: 使用路径规划算法计算出来的路径小车的详细运动指令、路径长度、小车最终朝向
        :return: 返回运动指令、路径长度、小车最终朝向
        """
        # 定义两个列表用于存储路径的所有x值和y值
        px = []
        py = []
        for x, y in path:
            px.append(x)
            py.append(y)

        # 如果只是计算距离, 不需要小车详细运动细节以及最终小车朝向, 那就仅仅返回路径长度
        if self.just_get_distance:
            return None, len(px) - 1, None

        # 获取小车详细运动细节以及小车最终的朝向, 这个朝向不一定是最终可用的朝向, 后续会进行修正
        directs, car_direct = self.get_directs(px, py)

        if self.plot:
            # 绘制路径
            plt.plot(px, py, linewidth=1)
            plt.plot(px, py, 'o')
        # 如果self.target_direct是字符串的话, 就是存储小车最终朝向, 如果路径规划后小车朝向与其不一致, 就需要添加运动指令（左转或者右转）, 修正小车最终朝向
        if car_direct != self.target_direct and isinstance(self.target_direct, str):
            last_distance = get_distance(directs[-1]['target_xy'][0], directs[-1]['target_xy'][1], self.target_direct)
            directs.append({
                'now_distance': directs[-1]['target_distance'],
                'target_distance': last_distance,
                'now_xy': directs[-1]['target_xy'],
                'target_xy': directs[-1]['target_xy']
            })
            List = ['上', '右', '下', '左']
            ori_index = List.index(car_direct)
            tar_index = List.index(self.target_direct)
            car_direct = self.target_direct
            if tar_index - ori_index in [-1, 3]:
                directs[-1]['move_mode'] = "左转"

            elif tar_index - ori_index in [1, -3]:
                directs[-1]['move_mode'] = "右转"
            else:
                print("这边出错了")
                exit()

        # 存储小车所有走过的坐标点, 用于可视化小车走完全程
        global total_point
        total_point += path + [car_direct]

        return directs, len(px) - 1, car_direct

    def get_directs(self, px, py):
        """

        方向：0是上。1是右。2是下。3是左
        :param px: 存储路径规划的路线中的所有x值
        :param py: 存储路径规划的路线中所有y值
        :return: 返回字典类型的小车详细运动指令以及字符串类型的小车最终朝向
        """
        directs = []
        car_direct = self.car_direct
        for i in range(len(px) - 1):

            if px[i] > px[i + 1]:
                # 向左
                if car_direct == "下":
                    directs.append({
                        "move_mode": "右转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "左"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '上':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "左"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '右':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "上"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], "上"),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "左"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                car_direct = "左"
            elif px[i + 1] > px[i]:
                # 向右
                if car_direct == "下":
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "右"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '上':
                    directs.append({
                        "move_mode": "右转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "右"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '左':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "下"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], "下"),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "右"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                car_direct = "右"
            elif py[i] > py[i + 1]:
                # 向下
                if car_direct == "左":
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "下"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })

                elif car_direct == '右':
                    directs.append({
                        "move_mode": "右转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "下"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '上':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "左"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], "左"),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "下"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                car_direct = "下"
            elif py[i + 1] > py[i]:
                # 向上
                if car_direct == "左":
                    directs.append({
                        "move_mode": "右转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "上"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '右':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "上"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '下':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "右"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], "右"),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "上"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])

                    })
                car_direct = "上"
            directs.append({
                "move_mode": "前进",
                # 获取当前位置小车前后左右的距离
                "now_distance": get_distance(px[i], py[i], car_direct),
                # 获取完成move_mode操作后小车前后左右的距离
                "target_distance": get_distance(px[i + 1], py[i + 1], car_direct),
                "now_xy": (px[i], py[i]),
                "target_xy": (px[i + 1], py[i + 1])
            })

        return directs, car_direct


def get_distance(now_x, now_y, car_direct):
    """
    获取小车当前坐标上右下左的距离。其中上是指小车的朝向位置
    返回的距离单位是白色块。0表示紧挨着墙壁。None表示那个方向没有任何障碍物
    :param now_x: x坐标
    :param now_y: y坐标
    :param car_direct: 小车朝向
    :return: 返回一个四元组。分别是上右下左的距离。
    """
    # 上右下左
    abs_distance = [None, None, None, None]
    for x, y in obs:
        if x == now_x:
            distance = (y - now_y)
            if distance > 0:
                distance = (distance - 1) // 2
                if abs_distance[0] is None or abs_distance[0] > distance:
                    abs_distance[0] = distance
            else:
                distance = ((-distance) - 1) // 2
                if abs_distance[2] is None or abs_distance[2] > (distance):
                    abs_distance[2] = distance
        elif y == now_y:
            distance = (x - now_x)
            if distance > 0:
                distance = (distance - 1) // 2
                if abs_distance[1] is None or abs_distance[1] > distance:
                    abs_distance[1] = distance
            else:
                distance = ((-distance) - 1) // 2
                if abs_distance[3] is None or abs_distance[3] > distance:
                    abs_distance[3] = distance

    if car_direct == '右':
        relate_distance = abs_distance[1:] + [abs_distance[0]]
    elif car_direct == '上':
        relate_distance = abs_distance

    elif car_direct == '下':
        relate_distance = abs_distance[2:] + abs_distance[:2]
    else:
        relate_distance = [abs_distance[-1]] + abs_distance[:-1]
    return tuple(relate_distance)


class Plotting:
    def __init__(self):
        pass

    def plot_grid(self, name):
        obs_x = [x[0] for x in obs]
        obs_y = [x[1] for x in obs]
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")


def get_path(s_start, s_goal, car_direct, plot, just_get_distance=False):
    """
    获取路径、路径长度、小车最终朝向
    :param s_start: 起点坐标
    :param s_goal: 终点坐标
    :param car_direct: 小车当前朝向
    :param plot: 是否画图
    :param just_get_distance: 是否是仅需要路径长度
    :return:
    """
    path, total_distance, car_direct = ADStar(s_start, s_goal, car_direct, plot=plot,
                                              just_get_distance=just_get_distance).run()
    return path, total_distance, car_direct


def get_distances(begin_point: tuple, target_point: tuple, mine_points: list) -> dict:
    """
    获取8个宝藏＋起点终点共10个点的两两之间的距离
    这是一个旅行商问题, 用于寻找全局最短路径, 由于宝藏数量较少, 所以解决这个旅行商问题使用的是暴力枚举法, 直接遍历所有可能的路径, 当然起点和终点是固定的
    :param begin_point: 起点坐标
    :param target_point:终点坐标
    :param mine_points:所有宝藏的坐标
    :return: 返回一个字典, 存放着所有边的距离
    """
    # 定义一个字典用于存储所有距离
    distance_dict = {}

    # 先计算起点和8个宝藏的距离, 计算终点和8个宝藏的距离
    for i in range(8):
        # 起点到每个点之间的距离
        distance_dict[(i, -1)] = \
            get_path(mine_points[i], begin_point, car_direct='上', plot=False, just_get_distance=True)[1]

        # 每个点到终点的距离
        distance_dict[(i, -2)] = \
            get_path(mine_points[i], target_point, car_direct='上', plot=False, just_get_distance=True)[1]

    # 计算8个宝藏两两之间的距离
    for i in range(7):
        for j in range(i + 1, 8):
            distance_dict[tuple(sorted([i, j]))] = \
                get_path(mine_points[i], mine_points[j], car_direct='上', plot=False, just_get_distance=True)[1]

    return distance_dict


def brute_force_tsp(begin_point: tuple, target_point: tuple, mine_points: list) -> tuple:
    """
    暴力枚举解决旅行商问题
    :param begin_point: 起点坐标
    :param target_point: 终点坐标
    :param mine_points: 所有宝藏的坐标
    :return: 返回一个元组类似(6, 5, 7, 4, 1, 3, 0, 2)
    """
    # 先获取所有距离
    distance_dict = get_distances(begin_point, target_point, mine_points)

    # 初始化最小距离为无穷大
    min_dist = float('inf')
    # 初始化最短路径为None
    shortest_path = None
    # 暴力枚举遍历所有可能的路径, 计算路径总长度,最终得到最短的路径
    for path in itertools.permutations(range(len(mine_points))):  # 遍历所有可能的排列，即所有可能的路径

        cur_dist = sum(distance_dict[tuple(sorted([path[i], path[i + 1]]))] for i in range(len(path) - 1))  # 计算该路径的总距离
        cur_dist += distance_dict[(path[0], -1)] + distance_dict[(path[-1], -2)]
        if cur_dist < min_dist:
            min_dist = cur_dist
            shortest_path = path  # 如果该路径小于min_dist，更新min_dist和shortest_path
    return shortest_path, min_dist


def get_total_path(begin_point: tuple, target_point: tuple, mine_points: list, shortest_path, car_direct="上") -> list:
    """

    :param begin_point: 起点坐标
    :param target_point: 终点坐标
    :param mine_points: 所有宝藏位置
    :param car_direct: 小车初始朝向
    :return:
    """

    # 定义一个列表存储所有路径。最终里面会有9个列表代表9条路线。每个列表里面有很多个字典存储小车详细运动指令
    total_path = []

    # 先获取起点和第一个宝藏之间的路径、路径长度、小车最终朝向
    path, total_distance, car_direct = get_path(begin_point, mine_points[shortest_path[0]], car_direct, plot=False)
    # 把得到的路径添加到列表中
    total_path.append(path)
    # 遍历所有宝藏得到每个宝藏到下一个宝藏之间的路径、路径长度和小车最终朝向
    for index in range(len(shortest_path) - 1):
        path, total_distance, car_direct = get_path(mine_points[shortest_path[index]],
                                                    mine_points[shortest_path[index + 1]], car_direct, plot=False)
        total_path.append(path)
    # 获取终点和第一个宝藏之间的路径、路径长度、小车最终朝向
    path, total_distance, car_direct = get_path(mine_points[shortest_path[-1]], target_point, car_direct, plot=False)
    total_path.append(path)

    return total_path


def vis(save_video=None, vis_img=None):
    """

    :param save_video: 是否保存为视频。如果None就是不保存。如果要保存的话这个参数就是保存的位置。注意不能有中文
    :return:
    """
    # 可视化用的图片
    if vis_img is None:
        vis_img = cv2.imread("./img/vis_labyrinth.jpg")

    if save_video is not None:
        if save_video.endswith(".avi") is False:
            exit(print("只能保存成avi格式。请修改.."))
        # fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # out = cv2.VideoWriter(save_video, fourcc, 10.0, (500, 500))
    # 获取每个白色块的宽度和高度
    each_y, each_x = vis_img.shape[0] // 10, vis_img.shape[1] // 10
    point_size = each_x * each_y // 800

    show_img = None
    for point in total_point:

        if isinstance(point, tuple):
            show_img = vis_img.copy()
            x, y = point
            x, y = each_x * ((x + 1) // 2) - each_x // 2, each_y * (11 - (y + 1) // 2) - each_y // 2

            cv2.circle(show_img, (x, y), point_size, (255, 0, 0), -1)

        else:

            if point == '左':
                cv2.line(show_img, (x, y), (x - each_x, y), (0, 255, 0), point_size)
            elif point == '右':
                cv2.line(show_img, (x, y), (x + each_x, y), (0, 255, 0), point_size)
            elif point == '下':
                cv2.line(show_img, (x, y), (x, y + each_y), (0, 255, 0), point_size)
            elif point == '上':
                cv2.line(show_img, (x, y), (x, y - each_y), (0, 255, 0), point_size)
            else:
                print("错误")
                exit()
            if save_video is not None:
                save_img = cv2.resize(show_img, (500, 500))
                for i in range(3):
                    out.write(save_img)
        if save_video is not None:
            out.write(cv2.resize(show_img, (500, 500)))
        # cv2.imshow("img", cv2.resize(show_img, (500, 500)))
        # cv2.waitKey(50)
    if save_video is not None:
        save_img = cv2.resize(show_img, (500, 500))
        for i in range(10):
            out.write(save_img)
        # out.release()


class MyEncoder(json.JSONEncoder):
    """
    这是为了解决把小车的详细运动指令写到json文件中报错的问题而重写JSONEncoder类。实际使用中可以去除。调试的时候可能用到
    """

    def default(self, obj):
        if isinstance(obj, np.int64):
            return int(obj)
        return json.JSONEncoder.default(self, obj)


def fine_tune_mine(begin_point, target_point, mine_points: list, optimize):
    """
    计算每个宝藏距离周围墙壁的距离, 偏移宝藏的坐标点以防止小车直接与宝藏相撞, 并且记录小车最终正确朝向
    :param mine_points: 所有宝藏位置
    :return:
    """
    # 后面需要修改宝藏位置, 所以先把原有宝藏位置存储下来
    ori_mine_points = mine_points.copy()
    if optimize is False:

        for i in range(len(mine_points)):
            dis = get_distance(mine_points[i][0], mine_points[i][1], "上")

            if dis[0] != 0:
                # 向上。y+1
                mine_points[i] = [(mine_points[i][0], mine_points[i][1] + 2), "下"]
            elif dis[1] != 0:
                # 向右 。x+1
                mine_points[i] = [(mine_points[i][0] + 2, mine_points[i][1]), "左"]
            elif dis[2] != 0:
                # 向下 y-1
                mine_points[i] = [(mine_points[i][0], mine_points[i][1] - 2), "上"]
            elif dis[3] != 0:
                # 向左 x-1
                mine_points[i] = [(mine_points[i][0] - 2, mine_points[i][1]), "右"]
            else:
                print("出错了")
                exit()
        # 把原有宝藏的位置也设置成墙壁, 防止小车穿越宝藏点
        obs.update(set(ori_mine_points))
        # 使用暴力枚举法解决旅行商问题求出小车前往的宝藏顺序
        shortest_path, min_dist = brute_force_tsp(begin_point, target_point, mine_points)
        global total_step
        total_step += min_dist
        return shortest_path, mine_points
    else:
        # 记录那些周围白色块不止一个的宝藏的索引以及它周围白色块的个数
        index_list = []
        # 记录一共有多少种宝藏摆放位置
        path_num = 1
        # 先修改那些周围只有一个白色块的宝藏的坐标
        for i in range(len(mine_points)):
            dis = get_distance(mine_points[i][0], mine_points[i][1], "上")
            white_block_num = 4 - dis.count(0)
            path_num *= white_block_num

            # print(mine_points)

            if white_block_num == 1:
                # 如果只有一个方向有白色块。那这个宝藏的偏移就固定了
                if dis[0] != 0:
                    # 向上。y+1
                    mine_points[i] = [(mine_points[i][0], mine_points[i][1] + 2), "下"]
                elif dis[1] != 0:
                    # 向右 。x+1
                    mine_points[i] = [(mine_points[i][0] + 2, mine_points[i][1]), "左"]
                elif dis[2] != 0:
                    # 向下 y-1
                    mine_points[i] = [(mine_points[i][0], mine_points[i][1] - 2), "上"]
                elif dis[3] != 0:
                    # 向左 x-1
                    mine_points[i] = [(mine_points[i][0] - 2, mine_points[i][1]), "右"]
            elif white_block_num == 0:
                print("出错了")
                exit()
            else:
                # print(i)
                index_list.append((i, dis))
        all_mine_points = []
        for num in range(path_num):
            all_mine_points.append(mine_points.copy())

        """
        一共有两个宝藏是周围有超过一个白色块的。all_mine_points列表里面的每一个列表都需要被修改两次
        """
        Dict = {}
        for i, dis in index_list:
            if dis[0] != 0:
                # 向上。y+1
                if Dict.get(i, False) is False:
                    Dict[i] = [[(mine_points[i][0], mine_points[i][1] + 2), "下", mine_points[i]]]
                else:
                    Dict[i].append([(mine_points[i][0], mine_points[i][1] + 2), "下", mine_points[i]])

            if dis[1] != 0:
                # 向右 。x+1
                if Dict.get(i, False) is False:
                    Dict[i] = [[(mine_points[i][0] + 2, mine_points[i][1]), "左", mine_points[i]]]
                else:
                    Dict[i].append([(mine_points[i][0] + 2, mine_points[i][1]), "左", mine_points[i]])

            if dis[2] != 0:
                # 向下 y-1
                if Dict.get(i, False) is False:
                    Dict[i] = [[(mine_points[i][0], mine_points[i][1] - 2), "上", mine_points[i]]]
                else:
                    Dict[i].append([(mine_points[i][0], mine_points[i][1] - 2), "上", mine_points[i]])

            if dis[3] != 0:
                # 向左 x-1
                if Dict.get(i, False) is False:
                    Dict[i] = [[(mine_points[i][0] - 2, mine_points[i][1]), "右", mine_points[i]]]
                else:
                    Dict[i].append([(mine_points[i][0] - 2, mine_points[i][1]), "右", mine_points[i]])

        # 首先将字典的值转换为一个列表
        values_list = list(Dict.values())
        index = 0
        # 求所有可能的配对
        pair_list = list(itertools.product(*values_list))

        for pair in pair_list:
            for ls in pair:
                all_mine_points[index][all_mine_points[index].index(ls[2])] = ls[:2]
            index += 1
        obs.update(set(ori_mine_points))
        cur_dist = float("inf")
        choose_path = None
        choose_point = None
        for points in all_mine_points:
            shortest_path, min_dist = brute_force_tsp(begin_point, target_point, points)

            if cur_dist > min_dist:
                cur_dist = min_dist
                choose_path = shortest_path
                choose_point = points
        total_step += cur_dist
        return choose_path, choose_point


def main(mine_points, begin_point=(19, 1), target_point=(1, 19), optimize=False):
    """

    :param mine_points: 所有宝藏的位置, 这里起点是左上角并且x和y单位是白色块
    :param begin_point: 起点坐标, 右下角
    :param target_point: 终点坐标, 左上角
    :param optimize: 是否进一步优化使得路径真正最短,这会导致运行速度变慢
    :return:
    """

    # 把宝藏位置转换成我们所需的正确的宝藏格式
    mine_points = [(x * 2 - 1, (11 - y) * 2 - 1) for (x, y) in mine_points]

    # 计算每个宝藏距离周围墙壁的距离, 偏移宝藏的坐标点以防止小车直接与宝藏相撞, 并且记录小车最终正确朝向
    shortest_path, mine_points = fine_tune_mine(begin_point, target_point, mine_points, optimize)

    total_path = get_total_path(begin_point, target_point, mine_points, shortest_path)

    plt.show()


def draw_text(img, use_time, step):
    # 设置文字样式
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1.0
    font_thickness = 2
    text_color = (0, 0, 255)  # 文字颜色为黑色
    center_x = 250
    center_y = 250
    # 计算文本绘制位置
    text1 = f"use_time:{use_time}s"
    text2 = f"step:{step}"
    text_width1, text_height1 = cv2.getTextSize(text1, font, font_scale, font_thickness)[0]
    text_width2, text_height2 = cv2.getTextSize(text2, font, font_scale, font_thickness)[0]
    x1 = center_x - text_width1 // 2
    y1 = center_y - text_height1 // 2 - text_height2
    x2 = center_x - text_width2 // 2
    y2 = center_y + text_height1 // 2

    # 绘制文本
    cv2.putText(img, text1, (x1, y1), font, font_scale, text_color, font_thickness)
    cv2.putText(img, text2, (x2, y2), font, font_scale, text_color, font_thickness)
    return img


if __name__ == '__main__':
    for opt in [False, True]:
        total_time = 0
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        total_step = 0
        # opt = False
        out = cv2.VideoWriter(f"./random_file/vis3{'long' if opt else 'short'}.avi", fourcc, 20.0, (500, 500))
        epoch = 100

        for i in range(epoch):
            print("\r进度：", i / 2, "%", end="", flush=True)
            total_point = []
            obs = get_obs()
            if opt is False:
                random_map = Treasure()
                img, mine_points = random_map.DrawTreasure("origin.jpg")
                cv2.imwrite(f"./random_file/random_img{i}.jpg", img)
                with open(f"./random_file/mine_points{i}.txt", "w") as f:
                    f.write(str(mine_points))
            else:
                img = cv2.imread(f"./random_file/random_img{i}.jpg")
                with open(f"./random_file/mine_points{i}.txt", "r") as f:
                    mine_points = eval(f.read())
            # mine_points = [(3, 3), (4, 5), (3, 6), (2, 9), (7, 6), (8, 5), (8, 8), (9, 2)]
            start = time()
            main(mine_points, optimize=opt)
            total_time += (time() - start)

            vis(save_video=f"vis3{'long' if opt else 'short'}.avi", vis_img=img)
        out.release()
        with open(f"./random_file/total_{'long' if opt else 'short'}.txt", "w") as file:
            file.write("avg_time:" + str(total_time / epoch) + " s\navg_step:" + str(total_step / epoch))
