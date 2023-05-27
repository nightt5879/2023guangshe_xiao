import cv2
import numpy as np
import math
from time import time, sleep
import matplotlib.pyplot as plt
import itertools
from multiprocessing import Process
import os
import json

# import heartrate
# heartrate.trace(browser=True)
# 可视化用的图片
vis_img = cv2.imread("./img/vis_labyrinth.jpg")
total_point = []
img = cv2.imread("./img/small_labyrinth.png", 0)
zero = np.where(img == 0)
obs = set((ys, 20 - xs) for xs, ys in zip(zero[0], zero[1]))


# 5秒左右
class Env:
    def __init__(self):
        self.x_range = 21  # size of background
        self.y_range = 21
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        return obs


class ADStar:
    def __init__(self, s_start, s_goal, car_direct, plot, just_get_distance=False, eps=2.5, heuristic_type="euclidean"):
        self.s_start, self.s_goal, self.car_direct = (s_start[0] if isinstance(s_start[-1], str) else s_start), (
            s_goal[0] if isinstance(s_goal[-1], str) else s_goal), car_direct

        self.target_direct = s_goal[1]
        self.heuristic_type = heuristic_type
        self.plot = plot
        self.Env = Env()  # class Env
        self.just_get_distance = just_get_distance
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.Plot = Plotting()
        self.Plot.obs = self.obs
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
        # self.Plot.plot_grid(self.title)
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
        px = []
        py = []
        for x, y in path:
            px.append(x)
            py.append(y)


        global total_point

        if self.just_get_distance:
            return None, len(px) - 1, None

        directs, car_direct = self.get_directs(px, py)

        if self.plot:
            plt.plot(px, py, linewidth=1)
            plt.plot(px, py, 'o')
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
        total_point += path + [car_direct]
        return directs, len(px) - 1, car_direct

    def get_directs(self, px, py):
        """
        方向：0是上。1是右。2是下。3是左
        :param px:
        :param py:
        :return:
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
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")


def get_path(s_start, s_goal, car_direct, plot, just_get_distance=False):
    path, total_distance, car_direct = ADStar(s_start, s_goal, car_direct, plot=plot,
                                              just_get_distance=just_get_distance).run()
    return path, total_distance, car_direct


def get_distances(begin_point, target_point, mine_points):
    """
    获取两两之间的距离
    :param begin_point:
    :param target_point:
    :param mine_points:
    :return:
    """

    distance_dict = {}

    # 优化前7.5秒左右 优化后5.6秒
    for i in range(8):
        # 起点到每个点之间的距离
        distance_dict[(i, -1)] = \
            get_path(mine_points[i], begin_point, car_direct='上', plot=False, just_get_distance=True)[1]

        # 每个点到终点的距离
        distance_dict[(i, -2)] = \
            get_path(mine_points[i], target_point, car_direct='上', plot=False, just_get_distance=True)[1]


    # 优化之前6秒
    for i in range(7):
        for j in range(i + 1, 8):
            if math.sqrt(
                    (mine_points[i][0][0] - mine_points[j][0][0]) ** 2 + (
                            mine_points[i][0][1] - mine_points[j][0][1]) ** 2) > 50:
                distance_dict[tuple(sorted([i, j]))] = 99999
            else:
                distance_dict[tuple(sorted([i, j]))] = \
                    get_path(mine_points[i], mine_points[j], car_direct='上', plot=False, just_get_distance=True)[1]

    return distance_dict


def brute_force_tsp(begin_point, target_point, points):
    # start = time()
    distance_dict = get_distances(begin_point, target_point, mine_points)

    # print(time() - start) # 5s左右

    min_dist = float('inf')  # 初始化min_dist为正无穷
    shortest_path = None
    for path in itertools.permutations(range(len(points))):  # 遍历所有可能的排列，即所有可能的路径

        cur_dist = sum(distance_dict[tuple(sorted([path[i], path[i + 1]]))] for i in range(len(path) - 1))  # 计算该路径的总距离
        cur_dist += distance_dict[(path[0], -1)] + distance_dict[(path[-1], -2)]
        if cur_dist < min_dist:
            min_dist = cur_dist
            shortest_path = path  # 如果该路径小于min_dist，更新min_dist和shortest_path
    # print(shortest_path)

    return shortest_path


def main(begin_point, target_point, mine_points):
    car_direct = "上"
    s_start = begin_point
    total_path = []
    shortest_path = brute_force_tsp(begin_point, target_point, mine_points)
    path, total_distance, car_direct = get_path(s_start, mine_points[shortest_path[0]], car_direct, plot=False)

    total_path.append(path)
    for index in range(len(shortest_path) - 1):
        path, total_distance, car_direct = get_path(mine_points[shortest_path[index]],
                                                    mine_points[shortest_path[index + 1]], car_direct, plot=False)
        total_path.append(path)
    path, total_distance, car_direct = get_path(mine_points[shortest_path[-1]], target_point, car_direct, plot=False)

    total_path.append(path)


    return total_path
class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.int64):
            return int(obj)
        return json.JSONEncoder.default(self, obj)

if __name__ == '__main__':
    start = time()
    begin_point = (19, 1)
    target_point = (1, 19)

    mine_points = [(3, 3), (4, 5), (3, 6), (2, 9), (7, 6), (8, 5), (8, 8), (9, 2)]
    mine_points = [(x * 2 - 1, (11 - y) * 2 - 1) for (x, y) in mine_points]
    ori_mine_points = mine_points.copy()
    # print(mine_points)
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

    obs.update(set(ori_mine_points))

    # start = time()
    total_path = main(begin_point, target_point, mine_points)
    print(total_path)
    print(len(total_path))
    print(time() - start)
    exit()
    # plt.show()


def vis():
    """
    可视化
    :return:
    """
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('vis2.avi', fourcc, 10.0, (500, 500))

    each_y, each_x = vis_img.shape[0] // 10, vis_img.shape[1] // 10
    last_point = None
    show_img = None
    for point in total_point:

        if isinstance(point, tuple):
            show_img = vis_img.copy()
            x, y = point
            x, y = each_x * ((x + 1) // 2) - each_x // 2, each_y * (11 - (y + 1) // 2) - each_y // 2

            cv2.circle(show_img, (x, y), 10, (255, 0, 0), -1)
            last_point = (x, y)
        else:

            if point == '左':

                cv2.line(show_img, last_point, (last_point[0] - each_x, last_point[1]), (0, 255, 0), 10)
            elif point == '右':
                cv2.line(show_img, last_point, (last_point[0] + each_x, last_point[1]), (0, 255, 0), 10)
            elif point == '下':
                cv2.line(show_img, last_point, (last_point[0], last_point[1] + each_y), (0, 255, 0), 10)
            elif point == '上':
                cv2.line(show_img, last_point, (last_point[0], last_point[1] - each_y), (0, 255, 0), 10)
            else:
                print("错误")
                exit()
            for i in range(3):
                out.write(cv2.resize(show_img, (500, 500)))

        out.write(cv2.resize(show_img, (500, 500)))
        # cv2.imshow("img", cv2.resize(show_img, (500, 500)))
        # cv2.waitKey(100)
    for i in range(10):
        out.write(cv2.resize(show_img, (500, 500)))
    out.release()


# vis()
