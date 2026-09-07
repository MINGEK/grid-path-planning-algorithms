"""Bidirectional‑A*算法"""
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
        self.coord = node  # 坐标对应numpy
        self.g_cost: float = 0.0  # 实际代价
        self.h_cost: float = 0.0  # 启发式估计代价
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


class Bidirectional_A_star:
    def __init__(self, robot_map: np.ndarray, source: Tuple[int, int], target: Tuple[int, int]):
        """导入 栅格地图、源节点、终节点"""
        self.grid = robot_map  # 栅格地图 numpy数组
        self.rows, self.cols = np.shape(robot_map)  # 栅格地图维度
        self.source = source  # 保存起点
        self.target = target  # 保存终点

    def distance(self, node1: Tuple[int, int], node2: Tuple[int, int]) -> float:
        """启发式距离函数 欧式距离"""
        return math.hypot(node1[0] - node2[0], node1[1] - node2[1])

    def get_path(self, end_node):
        path: List[Tuple[int, int]] = []
        cur: Optional[GridNode] = end_node
        while cur:
            path.append(cur.coord)
            cur = cur.parent
        return path[::-1]

    def get_valid_neighbors(self, current_coord: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        """返回当前点current点的有效邻接点（未排除已探索点）"""
        current_row, current_col = current_coord
        neighbors = []
        for d_row, d_col, step_cost in data.d_8:  # 从map导入d_8
            neighbor_row = current_row + d_row  # row
            neighbor_col = current_col + d_col  # col
            neighbor_coord = (neighbor_row, neighbor_col)
            # 检查边界和障碍物
            if 0 <= neighbor_row < self.rows and 0 <= neighbor_col < self.cols:
                # 先保证邻节点落在可通行区域上
                if self.grid[neighbor_coord] == 0:  # 0表示可通过
                    # 对角线移动需要检查两个正交方向
                    if step_cost > 1:
                        if self.grid[current_row][neighbor_col] == 0 and self.grid[neighbor_row][current_col] == 0:
                            neighbors.append((neighbor_coord, step_cost))
                    else:
                        neighbors.append((neighbor_coord, step_cost))
        return neighbors

    def search(self):
        # 检查起点终点是否有效
        if self.grid[self.source] == 1:
            print("错误：起点在障碍物上！或出界")
            return None
        if self.grid[self.target] == 1:
            print("错误：起点在障碍物上！或出界")
            return None

        source_node: GridNode = GridNode(self.source)  # 源节点
        target_node: GridNode = GridNode(self.target)  # 终节点

        # 正向
        fw_open_list: List[GridNode] = [source_node]
        fw_explored_set: Set[Tuple[int, int]] = set()
        fw_open_dict = {source_node.coord: source_node}

        # 反向
        bw_open_list: List[GridNode] = [target_node]
        bw_explored_set: Set[Tuple[int, int]] = set()
        bw_open_dict = {target_node.coord: target_node}
        #all
        all_explored_set : Set[Tuple[int, int]] = set()
        # 初始化交点
        meet_node = None

        while fw_open_list and bw_open_list:
            # ******************************************* 正向扩展********************************
            # 从OpenList中取出f_cost最小的节点作为当前节点
            fw_current_node = heapq.heappop(fw_open_list)
            # 若当前节点已在ClosedList中（说明已被更优路径扩展过），跳过
            if fw_current_node.coord in fw_explored_set:
                continue

            # 将当前节点加入ClosedList
            fw_explored_set.add(fw_current_node.coord)

            # 正向 查找 与反向的交点
            if fw_current_node.coord in bw_explored_set:
                meet_node = fw_current_node
                break

            for neighbor_coord, step_cost in self.get_valid_neighbors(fw_current_node.coord):

                # 若邻域节点已在ClosedList中，无需重复处理
                if neighbor_coord in fw_explored_set:
                    continue

                # 计算从起点经当前节点到达邻域节点的实际代价
                tentative_g = fw_current_node.g_cost + step_cost

                # 检查该邻域节点是否已在OpenList中
                existing = fw_open_dict.get(neighbor_coord)

                if existing is None or tentative_g < existing.g_cost:
                    # 发现更优路径：更新g_cost、parent指针，并重新加入OpenList
                    neighbor_node = GridNode(neighbor_coord)
                    neighbor_node.g_cost = tentative_g
                    neighbor_node.h_cost = self.distance(neighbor_coord, self.target)
                    neighbor_node.parent = fw_current_node
                    heapq.heappush(fw_open_list, neighbor_node)
                    fw_open_dict[neighbor_coord] = neighbor_node

            # ******************************************* 反向扩展********************************
            # 从OpenList中取出f_cost最小的节点作为当前节点
            bw_current_node = heapq.heappop(bw_open_list)
            # 若当前节点已在ClosedList中（说明已被更优路径扩展过），跳过
            if bw_current_node.coord in bw_explored_set:
                continue

            # 将当前节点加入ClosedList
            bw_explored_set.add(bw_current_node.coord)

            # 反向 查找 与正向的交点
            if bw_current_node.coord in fw_explored_set:
                meet_node = bw_current_node
                break


            for neighbor_coord, step_cost in self.get_valid_neighbors(bw_current_node.coord):

                # 若邻域节点已在ClosedList中，无需重复处理
                if neighbor_coord in bw_explored_set:
                    continue

                # 计算从起点经当前节点到达邻域节点的实际代价
                tentative_g = bw_current_node.g_cost + step_cost

                # 检查该邻域节点是否已在OpenList中
                existing = bw_open_dict.get(neighbor_coord)

                if existing is None or tentative_g < existing.g_cost:
                    # 发现更优路径：更新g_cost、parent指针，并重新加入OpenList
                    neighbor_node = GridNode(neighbor_coord)
                    neighbor_node.g_cost = tentative_g
                    neighbor_node.h_cost = self.distance(neighbor_coord, self.source)
                    neighbor_node.parent = bw_current_node
                    heapq.heappush(bw_open_list, neighbor_node)
                    bw_open_dict[neighbor_coord] = neighbor_node

            all_explored_set = fw_explored_set | bw_explored_set

            yield all_explored_set

        if meet_node is None:
            return None  # 没有通路

        # 正向路径
        path_fw = self.get_path( fw_open_dict[meet_node.coord] )

        # 反向路径
        path_bw = self.get_path( bw_open_dict[meet_node.coord] )
        path_bw.reverse()
        # 拼接，去掉重复的相遇点
        all_path = path_fw +  path_bw[1:]

        return all_path, all_explored_set

if __name__ == "__main__":
    Bidirectional_A_star_ = Bidirectional_A_star(robot_map=data.np_map, source=data.source, target=data.target)

    viz = dynamic.MapVisualizer()
    viz.plot_dynamic(Bidirectional_A_star_.search, title="Bidirectional-A* Path Planning Dynamic Result")
