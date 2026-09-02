"""A*算法"""
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
    def __init__(self, node: Tuple[int,int]):
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

class A_star:
    target: tuple[int, int]

    def __init__(self, robot_map: np.ndarray, source:Tuple[int,int], target:Tuple[int,int]):
        """导入 栅格地图、源节点、终节点, 初始化 最短路径 已探索节点"""
        self.grid = robot_map  # 栅格地图 numpy数组
        self.rows, self.cols = np.shape(robot_map)  # 栅格地图维度
        self.source = source  # 保存起点
        self.target = target  # 保存终点

    def heuristic(self, neighbor_coord :Tuple[int, int]) -> float:
        """启发式距离函数 欧式距离"""
        return math.hypot(neighbor_coord[0] - self.target[0], neighbor_coord[1] - self.target[1])

    def get_valid_neighbors(self, current_coord: Tuple[int, int]) -> List[Tuple[Tuple[int,int], float]]:
        """返回当前点current点的有效邻接点（未排除已探索点）"""
        current_x, current_y = current_coord
        neighbors = []
        for dx, dy, step_cost in data.d_8:  # 从map导入d_8
            neighbor_x = current_x + dx #row
            neighbor_y = current_y + dy #col
            neighbor_coord = (neighbor_x, neighbor_y)
            # 检查边界和障碍物
            if 0 <= neighbor_x < self.rows and 0 <= neighbor_y < self.cols:
                #先保证邻节点落在可通行区域上
                if self.grid[neighbor_coord] == 0:  # 0表示可通过
                    # 对角线移动需要检查两个正交方向
                    if step_cost > 1:
                        if self.grid[current_x][neighbor_y] == 0 and self.grid[neighbor_x][current_y] == 0:
                            neighbors.append((neighbor_coord, step_cost))
                    else:
                        neighbors.append((neighbor_coord, step_cost))
        return neighbors

    def plan(self):
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
        # 将起点加入 存放字典的形式
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

            for neighbor_coord, step_cost in self.get_valid_neighbors(current_node.coord):
                # 取出当前点所有有效邻接点
                neighbor_node = GridNode(neighbor_coord)
                # 若邻域节点已在ClosedList中，无需重复处理
                if neighbor_coord in explored_set:
                    continue

                # 计算从起点经当前节点到达邻域节点的实际代价
                tentative_g = current_node.g_cost + step_cost

                # 检查该邻域节点是否已在OpenList中
                existing = open_dict.get(neighbor_coord)

                if existing is None or tentative_g < existing.g_cost:
                    # 发现更优路径：更新g_cost、parent指针，并重新加入OpenList
                    neighbor_node.g_cost = tentative_g
                    neighbor_node.h_cost = self.heuristic(neighbor_coord)  # 使用self.target
                    neighbor_node.parent = current_node
                    heapq.heappush(open_list, neighbor_node)
                    open_dict[neighbor_coord] = neighbor_node
        # OpenList为空且未到达终点，说明无可行路径
        return None

if __name__ == "__main__":
    flag_static, flag_dynamic = 0, 1
    if flag_static:
        pass
    elif flag_dynamic:
        Astar_ = A_star(robot_map=data.np_map, source=data.source, target=data.target)
        viz = dynamic.MapVisualizer()
        viz.plot_dynamic(Astar_.plan, title="A* Path Planning Dynamic Result")
