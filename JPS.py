"""JPS算法"""

import grid_map_data as data
import plt_dynamic as dynamic
import heapq
import math
import numpy as np
from typing import List, Tuple, Optional, Set


class GridNode:
    """
    栅格节点类：存储每个地图格子的坐标与路径搜索所需的代价信息
    Attributes:
        x, y: 节点在地图中的坐标索引
        g_cost: 从起点到当前节点的实际代价（累加步长）
        h_cost: 从当前节点到终点的启发式估计代价（欧几里得距离）
        parent: 当前节点在搜索树中的父节点，用于最终回溯路径
    """
    def __init__(self, node: Tuple[int, int]):
        self.coord = node              # 坐标对应numpy
        self.g_cost: float = 0.0      # 实际代价
        self.h_cost: float = 0.0      # 启发式估计代价
        self.parent: Optional['GridNode'] = None

    @property
    def f_cost(self) -> float:
        """
        总评估代价 f = g + h
        f_cost越小，说明该节点越有可能位于最优路径上
        """
        return self.g_cost + self.h_cost

    def __lt__(self, other: 'GridNode') -> bool:
        """
        定义节点间的小于比较，用于heapq优先队列排序
        当f_cost相同时，比较h_cost以保证搜索的稳定性
        """
        if self.f_cost == other.f_cost:
            return self.h_cost < other.h_cost
        return self.f_cost < other.f_cost

    def __eq__(self, other: object) -> bool:
        """基于坐标判断节点是否相同"""
        if not isinstance(other, GridNode):
            return False
        return self.coord == other.coord

    def __hash__(self) -> int:
        """基于坐标生成哈希值，支持将节点存入集合（Set）"""
        return hash(self.coord)


class JPS:
    target: tuple[int, int]

    def __init__(self, robot_map: np.ndarray, source: Tuple[int, int], target: Tuple[int, int]):
        """导入 栅格地图、源节点、终节点, 初始化 最短路径 已探索节点"""
        self.grid = robot_map  # 栅格地图 numpy数组
        self.rows, self.cols = np.shape(robot_map)  # 栅格地图维度
        self.source = source  # 保存起点
        self.target = target  # 保存终点

    def heuristic(self, neighbor_coord :Tuple[int, int]) -> float:
        """启发式距离函数 欧式距离"""
        return math.hypot(neighbor_coord[0] - self.target[0], neighbor_coord[1] - self.target[1])

    # 检测 点 是否可通行
    def is_valid_node(self, node: Tuple[int, int]) -> bool:
        row, col = node
        return 0 <= row < self.rows and 0 <= col < self.cols and self.grid[row, col] == 0

    # 检测 对于当前点坐标，（d_row, d_col)方向，是否存在强迫邻居
    def has_forced_neighbor(self, current_coord: Tuple[int, int], d_row, d_col) -> bool:
        """
        判断当前点是否有强迫邻居 row,col
        当前点的前导节点为 （row-d_row, col-d_col）
        """
        current_row, current_col = current_coord
        # 当前点的前导节点 parent_x = current_row - d_row, parent_col = current_col - d_col
        # 考虑AGV 斜线通行条件 的 横向移动
        if d_row == 0 and d_col != 0:
            # 依据AGV斜向通行条件，修改了横向移动的判断条件
            if not self.is_valid_node((current_row - 1, current_col - d_col)):
                return True
            if not self.is_valid_node((current_row + 1, current_col - d_col)):
                return True

        # 直线 纵向 探索  d_col = 0
        elif d_row != 0 and d_col == 0:
            # 依据AGV斜向通行条件，修改了纵向移动的判断条件
            if not self.is_valid_node((current_row - d_row, current_col - 1)):
                return True
            if not self.is_valid_node((current_row - d_row, current_col + 1)):
                return True

        # 斜向探索 右下斜向左上探索 d_row = -1，d_col = -1， 左上斜向右下探索 d_row = 1 ，d_col = 1
        """
        elif d_row == d_col:
            # 可能有两个位置 如果有障碍物出现 会产生强迫邻居
            node1 = (current_row - d_row, current_col)
            node1_force = (current_row - d_row, current_col + d_col)
            node2 = (current_row, current_col - d_col)
            node2_force = (current_row + d_row, current_col - d_col)
            # 如果其中有一个存在障碍物，则 视为有强迫邻居
            if not self.is_valid_node(node1) and self.is_valid_node(node1_force):
                return True
            if not self.is_valid_node(node2) and self.is_valid_node(node2_force):
                return True
        # 右上斜向左下探索 d_row = 1, d_col =-1    左下斜向右上探索 d_row = -1,d_col = 1
        elif d_row + d_col == 0:
            # 可能有两个位置 如果有障碍物出现 会产生强迫邻居
            node1 = (current_row - d_row, current_col)
            node1_force = (current_row - d_row, current_col + d_col)
            node2 = (current_row, current_col - d_col)
            node2_force = (current_row + d_row, current_col - d_col)
            # 如果其中有一个存在障碍物，则 视为有强迫邻居
            if not self.is_valid_node(node1) and self.is_valid_node(node1_force):
                return True
            if not self.is_valid_node(node2) and self.is_valid_node(node2_force):
                return True
            """

        return False

    # 对 由前导节点至当前节点 进行 剪枝
    def get_pruned_directions(self,
                             current_coord: Tuple[int, int],
                             parent_coord: Tuple[int, int]) -> List[Tuple[int, int]]:
        """论文 §3 Pruning Rules + 严格 corner-cut 统一筛选"""
        # 如果其前导节点不存在（如起点），则八方向探索
        if parent_coord is None:
            raw = [(d_row, d_col) for d_row, d_col, _ in data.d_8]
        # 否则 依据前导节点来推理 当前节点的有效方向
        else:
            # 求取方向向量
            c_row, c_col = current_coord
            p_row, p_col = parent_coord
            _d_row = c_row - p_row
            _d_col = c_col - p_col
            # 转化为 方向 单位向量 (非标准，如果（5,4）转化为（1,1）)
            d_row = 0 if _d_row == 0 else int(_d_row / abs(_d_row))
            d_col = 0 if _d_col == 0 else int(_d_col / abs(_d_col))
            # 建立剪枝后的方向集合
            raw = []
            # 斜线探索 一共有八个方向，我对标准JPS剪枝做了修改，削减离前导节点最近的三个方向，保留至五个方向，有效直线方向不受障碍物影响（我这里考虑的是AGV斜向通行，双侧无切角）
            if d_row != 0 and d_col != 0:
                # ====== 自然方向(规则 2) ======
                # 仅当斜线两侧均 无障碍物时，保留斜线本身
                raw.append((d_row, 0))  # 正交分量 1
                raw.append((0, d_col))  # 正交分量 2
                # 同方向 斜向
                raw.append((d_row, d_col))  # 斜向本身

                # ====== 强迫邻居方向(规则 3) ======
                #raw.append((-d_row, d_col))
                #raw.append((d_row, -d_col))

            # 直线横向探索 保留五个方向
            elif d_row == 0 and d_col !=0:
                # 原来方向的横向移动
                raw.append((d_row, d_col))
                # 依据AGV斜向通行条件而加入的 两个垂直方向
                raw.append((-1, 0))
                raw.append((1, 0))
                # 斜线分支1
                raw.append((-1, d_col))
                # 斜线分支2
                raw.append((1, d_col))
            # 直线纵向探索
            elif d_row != 0 and d_col == 0:
                #原有方向
                raw.append((d_row, d_col))
                # 我自己加入的垂直向
                raw.append((0, -1))
                raw.append((0, 1))

                raw.append((d_row, -1))
                raw.append((d_row, 1))

        return raw

    # 查询 当前节点 沿着dx、dy方向 移动后， 所有有效跳点 是否存在跳点，如果没有就继续移动，直到下一个移动点非法或者到达终点 ，其中调用了 检测强迫邻居函数
    def jump(self, current_coord: Tuple[int, int], d_row: int, d_col: int) -> Optional[Tuple[int, int]]:
        """
        返回周围邻节点存在 强迫邻节点 的 节点, 即跳点
        跳点定义，除了起点、终点以外，如果该点在确定父节点的情况下，周围节点依据 强迫邻居定义 存在有 强迫邻居节点，则将此点认定为跳点
        代入 当前current(x,y) 以及可以确定其父节点的（dx，dy），父节点实际为（x-dx。y-dy）
        """
        c_row, c_col = current_coord
        while True:
            # n_row,n_col 是当前节点 沿着dx、dy移动一个单位向量产生的新点
            n_row = c_row + d_row
            n_col = c_col + d_col
            # 1.撞墙/越界/严格 corner-cut，终止这条射线
            if not self.is_valid_node((n_row,n_col)):
                return None
            # AGV斜线通行 两侧无障碍物
            if d_row !=0 and d_col !=0:
                if (not self.is_valid_node((n_row,c_col))) or (not self.is_valid_node((c_row,n_col))):
                    return None
            # 2.到达终点，直接返回终点作为跳点
            if (n_row, n_col) == self.target:
                return n_row, n_col
            # 3.发现强迫邻居，当前点就是跳点
            if self.has_forced_neighbor((n_row, n_col), d_row, d_col):
                print("ces",n_row, n_col)
                return n_row, n_col
            # 4.斜向：压入水平、竖直子探测任务
            if d_row != 0 and d_col != 0:
                # 先求解纵向再求解横向
                if self.jump((n_row, n_col), d_row, 0) is not None:
                    return n_row, n_col
                if self.jump((n_row, n_col), 0, d_col) is not None:
                    return n_row, n_col
            # 继续向前走同一个方向
            print(n_row, n_col)
            c_row, c_col = n_row, n_col
        return None

    # 查询在有前导节点的当前节点情况下 所有有效的跳点
    def get_successors(self, current_coord: Tuple[int, int], parent_coord: Optional[Tuple[int, int]]) -> List[Tuple[Tuple[int, int], bool]]:
        """获取跳点后继集合，使用剪枝后的方向；jump 失败则退化单步"""
        successors: List[Tuple[Tuple[int, int], bool]] = []
        c_r, c_c = current_coord

        dirs = self.get_pruned_directions(current_coord, parent_coord)

        for d_row, d_col in dirs:
            # dir 不进行判断 移动的后的点是否满足条件，由这里 进行跳点探索前完成
            # if 判断 如果移动后的点是障碍物则迭代下一个
            if not self.is_valid_node((c_r+d_row,c_c+d_col)):
                continue
            if d_row !=0 and d_col !=0:
                if (not self.is_valid_node((c_r+d_row,c_c))) or (not self.is_valid_node((c_r,c_c+d_col))):
                    continue
            jp = self.jump(current_coord, d_row, d_col)
            if jp is not None:
                print(jp,"p:",current_coord, d_row, d_col)
                successors.append((jp, True))
        return successors

    # 主函数
    def search(self):
        # 检查起点终点是否有效
        if self.grid[self.source] == 1:
            print("错误：起点在障碍物上！或出界")
            return None
        if self.grid[self.target] == 1:
            print("错误：起点在障碍物上！或出界")
            return None

        source_node: GridNode = GridNode(self.source)  # 源节点

        open_list: List[GridNode] = [source_node]
        # 记录已扩展的节点，避免重复处理
        explored_set: Set[Tuple[int, int]] = set()
        # 用于快速查找OpenList中是否已存在某节点，并支持更新更优路径
        open_dict = {source_node.coord: source_node}  # 使用source_node

        while open_list:

            # 从OpenList中取出f_cost最小的节点作为当前节点
            current_node = heapq.heappop(open_list)
            # 若当前节点已在ClosedList中（说明已被更优路径扩展过），跳过
            if current_node.coord in explored_set:
                yield explored_set
                continue

            # 将当前节点加入ClosedList
            explored_set.add(current_node.coord)

            yield explored_set

            # 到达目标点：通过parent指针回溯，重构完整路径
            if current_node.coord == self.target:
                path = []
                node = current_node
                while node is not None:
                    path.append(node.coord)
                    node = node.parent
                return path[::-1], explored_set  # 反转列表，得到从起点到终点的顺序

            # 取出当前点的 前导节点，因为判断当前点的后继跳点，依赖于它的前导节点来控制方向
            parent_node = current_node.parent

            if parent_node is None:
                parent_coord = None
            else:
                parent_coord = parent_node.coord

            # 查询在当前节点下，前导点的状况下 的 所有有效后继节点
            successors = self.get_successors(current_node.coord, parent_coord)
            print("c:", successors)
            for succ_coord, is_jump in successors:
                # 记录后继跳点信息，并实例化后，压入最优队列中
                dr = succ_coord[0] - current_node.coord[0]
                dc = succ_coord[1] - current_node.coord[1]
                cost = math.hypot(dr, dc)

                tentative_g = current_node.g_cost + cost

                existing = open_dict.get(succ_coord)

                if existing is None or tentative_g < existing.g_cost:
                    successor_node = GridNode(succ_coord)
                    successor_node.g_cost = tentative_g
                    successor_node.h_cost = self.heuristic(succ_coord)
                    successor_node.parent = current_node
                    heapq.heappush(open_list, successor_node)
                    open_dict[succ_coord] = successor_node

        return None, explored_set

if __name__ == "__main__":
    JPS_ = JPS(robot_map=data.np_map, source=data.source, target=data.target)
    flag_static, flag_dynamic = 0, 1
    if flag_static:
        pass
    elif flag_dynamic:
        viz = dynamic.MapVisualizer()
        viz.plot_dynamic(JPS_.search, title="JPS Path Planning Dynamic Result")
