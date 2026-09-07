"""
RRT-connect算法
步长为 2 ，换算后直线方向 移动 最大为 2，斜线方向 移动 最大为 1
"""
import grid_map_data as data

import plt_dynamic as dynamic
import math
import numpy as np
from typing import List, Tuple, Optional, Set


# 这个算法 不用写入距离代价
class GridNode:
    def __init__(self, coord: Tuple[int, int]):
        self.coord = coord
        self.parent = None


class RRT_connect:
    def __init__(self, robot_map: np.ndarray,
                 source: Tuple[int, int],
                 target: Tuple[int, int],
                 random_seed: Optional[int] = 42):
        self.grid = robot_map
        self.rows, self.cols = np.shape(robot_map)  # 栅格地图维度
        self.source = source
        self.target = target
        self.rng = np.random.default_rng(random_seed)

    @staticmethod
    def distance(node1: Tuple[int, int], node2: Tuple[int, int]) -> float:
        return math.hypot(node1[0] - node2[0], node1[1] - node2[1])

    # 随机点生成
    def generate_rnd(self) -> Tuple[int, int]:
        # 10%概率直接采样目标点，加快收敛速度
        if self.rng.random() < 0.1:
            rnd = self.target
        else:
            # 随机在栅格范围内采一个整数坐标点，可以保证在栅格范围内
            row = self.rng.integers(0, self.rows)
            col = self.rng.integers(0, self.cols)
            rnd = (int(row), int(col))
        return rnd

    # 有效点检测
    def is_valid_node(self, node: Tuple[int, int]) -> bool:
        row, col = node
        return 0 <= row < self.rows and 0 <= col < self.cols and self.grid[row, col] == 0

    # 传入对象 node_list 为对象。rnd为元组, 输出对象
    # 在节点列表里找到离随机采样点最近的节点
    def nearest_node(self, tree_nodes: List[GridNode], rnd: Tuple[int, int]) -> GridNode:
        # 默认第一个节点为最近点
        nearest = tree_nodes[0]
        # 初始化最小距离
        min_d = self.distance(nearest.coord, rnd)
        # 遍历所有节点挨个比较
        for tree_node in tree_nodes:
            d = self.distance(tree_node.coord, rnd)
            if d < min_d:
                min_d = d
                nearest = tree_node
        return nearest

    # 从node朝着rnd方向，走固定步长生成新节点，由于步长为2，且坐标向下取整
    def steer(self, node: Tuple[int, int], dir_rnd: Tuple[int, int], step=2) -> Tuple[int, int]:

        d = self.distance(node, dir_rnd)
        d_row = (dir_rnd[0] - node[0]) / d
        d_col = (dir_rnd[1] - node[1]) / d
        if d <= step:
            return dir_rnd
        # 沿row角度前进step，算出新row坐标，保证新点坐标取整 靠近 出发点
        if dir_rnd[0] > node[0]:
            n_row = node[0] + math.floor(step * d_row)
        else:
            n_row = node[0] + math.ceil(step * d_row)

        # 沿cel角度前进step，算出新col坐标，保证新点坐标取整 靠近 出发点
        if dir_rnd[1] > node[1]:
            n_col = node[1] + math.floor(step * d_col)
        else:
            n_col = node[1] + math.ceil(step * d_col)

        # 返回新建的节点对象
        return n_row, n_col

    # 判断两点之间的连线 是否有效
    def is_valid_line(self, node1: Tuple[int, int], node2: Tuple[int, int]) -> bool:
        # 根据线段长度确定采样多少个点检查碰撞
        steps = self.distance(node1, node2)
        # 沿着线段均匀取点逐个检测
        if steps == math.sqrt(2):
            if self.grid[(node1[0], node2[1])] == 1 or self.grid[(node2[0], node1[1])] == 1:
                return False
        else:
            for i in range(int(steps) + 1):
                t = i / steps
                # 线性插值得到当前检测点坐标
                x = int(node1[0] * (1 - t) + node2[0] * t)
                y = int(node1[1] * (1 - t) + node2[1] * t)
                # 防止坐标越界
                if not self.is_valid_node((x, y)):
                    return False
        # 整条线段都没有碰到障碍物
        return True

    # 从终点节点不断回溯parent，拼接出完整路径
    def get_path(self, end_node):
        path: List[Tuple[int, int]] = []
        cur: Optional[GridNode] = end_node
        while cur:
            path.append(cur.coord)
            cur = cur.parent
        return path[::-1]

    def extend(self, tree:List[GridNode], rnd:Tuple[int, int])-> tuple[None, bool] | tuple[GridNode, bool]:
        # 拓展步，已保证返回节点有效
        near_node = self.nearest_node(tree, rnd)
        if rnd == near_node.coord:
            return None, False

        new_coord = self.steer(near_node.coord, rnd)

        if not self.is_valid_node(new_coord) or not self.is_valid_line(near_node.coord, new_coord):
            return None, False

        # 此时已拓展城市，实例化新节点，并为其添加父节点
        new_node = GridNode(new_coord)
        tree.append(new_node)
        new_node.parent = near_node
        return new_node, True

    def connect(self, tree:List[GridNode], reach_coord: Tuple[int, int]
                )->tuple[None, bool] | tuple[GridNode, bool]:
        # 如果当前树的距离 目标点，的最近节点 相等，则直接返回链接成功
        near_node = self.nearest_node(tree, reach_coord)
        if near_node.coord == reach_coord:
            return near_node, True
        while True:
            # 疯狂拓展 朝目标方向拓展，直到链接 或 节点失效
            status, flag = self.extend(tree, reach_coord)
            # 拓展失败，直接退出
            if not flag:
                break
            # 如果到达目标点，直接返回状态
            if status.coord == reach_coord:
                return status, True
            # 在没有失败和 到达目标点的情况下，一直拓展
            tree.append(status)
        return status, False

    # 栅格地图版本 RRT 主算法函数
    def search(self, max_iter=5000):
        # 检查起点终点是否有效
        if self.grid[self.source] == 1:
            print("错误：起点在障碍物上！或出界")
            return None
        if self.grid[self.target] == 1:
            print("错误：起点在障碍物上！或出界")
            return None

        source_node = GridNode(self.source)
        target_node = GridNode(self.target)

        s_tree: List[GridNode] = [source_node] # 源点树
        t_tree: List[GridNode] = [target_node] # 终点树

        # 初始化
        a_tree: List[GridNode] = s_tree # 源点树
        b_tree: List[GridNode] = t_tree # 终点树
        # 将当前节点加入ClosedList
        explored_set: Set[Tuple[int, int]] = set()

        # 开始迭代生长随机树
        i_iter = 0
        for _ in range(max_iter):
            i_iter += 1
            # 生成随机点
            rnd = self.generate_rnd()

            a_new_node, flag = self.extend(a_tree, rnd)
            # 拓展失败则，则跳过
            if not flag:
                continue

            status, flag = self.connect(b_tree, a_new_node.coord)

            explored_set = set([tn.coord for tn in a_tree] + [tn.coord for tn in b_tree])
            yield explored_set
            # 如果找到链接点，则查询完整路径
            if flag:
                # 来判断哪个树是起点树
                if a_tree[0].coord == self.source:
                    s_tree = a_tree
                    t_tree = b_tree
                else:
                    t_tree = a_tree
                    s_tree = b_tree

                path  =  self.get_path(a_tree[-1]) + self.get_path(b_tree[-1])[::-1][1:]
                return path, None

            a_tree, b_tree = b_tree, a_tree
        # 迭代用完也没找到路径
        return None


# ---------------- 程序入口，测试运行 ----------------
if __name__ == "__main__":
    RRT_connect_ = RRT_connect(robot_map=data.np_map, source=data.source, target=data.target)

    viz = dynamic.MapVisualizer()
    viz.plot_dynamic(RRT_connect_.search, title="RRT-connect Path Planning Dynamic Result")
