"""RRT* 算法"""

import math
import numpy as np
from typing import List, Tuple, Optional
import grid_map_data as data
import plt_dynamic as dynamic

class GridNode:
    def __init__(self, coord: Tuple[int, int]):
        self.coord: Tuple[int, int] = coord
        self.parent: Optional["GridNode"] = None
        self.g_cost: Optional[float] = None


class RrtStar:
    """RRT* Rewire"""
    DIM      = 2        # 2D 栅格
    GAMMA    = 10.0     # 邻居半径系数 γ，论文要求 γ > 2(1+1/d)^(1/d) · μ(X free)/ζ_d
    R_MAX    = 5.0      # 邻居半径上限；step=2，建议 3~5 倍
    GOAL_BIAS = 0.10    # 10% 概率直接采样目标，加速收敛
    STEP     = 2        # steer 单步 长度（栅格单位）

    def __init__(self,
                 robot_map: np.ndarray,
                 source: Tuple[int, int],
                 target: Tuple[int, int],
                 random_seed: Optional[int] = 42):
        self.grid = robot_map
        self.rows, self.cols = robot_map.shape
        self.source = source
        self.target = target
        self.rng = np.random.default_rng(random_seed)

    @staticmethod
    def distance(node1: Tuple[int, int], node2: Tuple[int, int]) -> float:
        return math.hypot(node1[0] - node2[0], node1[1] - node2[1])

    # 随机点生成
    def generate_rnd(self) -> Tuple[int, int]:
        # 10%概率直接采样目标点，加快收敛速度
        if self.rng.random() < self.GOAL_BIAS:
            rnd = self.target
        else:
            # 随机在栅格范围内采一个整数坐标点，可以保证在栅格范围内
            row = self.rng.integers(0, self.rows)
            col = self.rng.integers(0, self.cols)
            rnd = (row, col)
        return rnd

    def is_valid_node(self, coord: Tuple[int, int]) -> bool:
        row, col = coord
        return (0 <= row < self.rows) and (0 <= col < self.cols) and (self.grid[row, col] == 0)

    def nearest_node(self,
                     tree_nodes: List[GridNode],
                     rnd: Tuple[int, int]) -> GridNode:
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

    def steer(self,
              node: Tuple[int, int],
              dir_rnd: Tuple[int, int],
              step: Optional[int] = None) -> Tuple[int, int]:

        if step is None:
            step = self.STEP

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

    def is_valid_line(self,
                      node1: Tuple[int, int],
                      node2: Tuple[int, int]) -> bool:
        """两点连线的碰撞检测：
           · 对角线长度 √2 时严格使用切角约束
           · 其他长度沿线段线性插值采样所有途经整数点"""

        d = self.distance(node1, node2)
        # 纯对角一步：切角约束（你原代码保留的好习惯）
        if abs(node1[0] - node2[0]) == 1 and abs(node1[1] - node2[1]) == 1:
            if (not self.is_valid_node((node1[0], node2[1]))
                    or not self.is_valid_node((node2[0], node1[1]))):
                return False

        samples = max(1, int(math.ceil(d)))  # 至少 1 个采样
        for i in range(samples + 1):
            t = 0.0 if samples == 0 else i / samples
            x = int(round(node1[0] * (1.0 - t) + node2[0] * t))
            y = int(round(node1[1] * (1.0 - t) + node2[1] * t))
            if not self.is_valid_node((x, y)):
                return False
        return True

    def rewire(self, tree_nodes: List[GridNode], new_node: GridNode) -> bool:
        new_coord = new_node.coord
        n = len(tree_nodes)
        # ---- 0) 邻居半径 r：n<=1 时 log(1)=0，必须兜底 ----
        if n <= 1:
            r = self.R_MAX
        else:
            r = min(self.GAMMA * (math.log(n) / n) ** (1.0 / self.DIM), self.R_MAX)

        # 找出 新节点 r半径内 满足要求的所有节点
        neighbors: List[GridNode] = [
            tn for tn in tree_nodes
            if self.distance(tn.coord, new_coord) <= r
        ]

        if not neighbors: return False

        best_parent: Optional[GridNode] = None
        best_cost = math.inf
        for tn in neighbors:

            if not self.is_valid_line(tn.coord, new_coord): continue

            cand_cost = tn.g_cost + self.distance(tn.coord, new_coord)

            if cand_cost < best_cost:
                best_cost  = cand_cost
                best_parent = tn

        if best_parent is None:
            return False

        new_node.parent = best_parent
        new_node.g_cost = best_cost

        # ---- 3) Rewire：回头看看邻居能不能"从 new_node 绕一下"更便宜 ----
        for tn in neighbors:
            if tn is best_parent:
                continue                 # 爹不需要再检查
            if not self.is_valid_line(new_coord, tn.coord):
                continue
            alternative = new_node.g_cost + self.distance(new_coord, tn.coord)
            if alternative < tn.g_cost:
                tn.g_cost = alternative  # 新代价
                tn.parent = new_node     # 改父指针 → 整条 子树路径被间接变优

        return True

    def get_path(self, end_node: GridNode) -> List[Tuple[int, int]]:
        path: List[Tuple[int, int]] = []
        cur: Optional[GridNode] = end_node
        while cur:
            path.append(cur.coord)
            cur = cur.parent
        return path[::-1]

    def plan(self, max_iter: int = 5000):
        source_node = GridNode(self.source)
        source_node.g_cost = 0.0
        tree_nodes: List[GridNode] = [source_node]

        target_node = GridNode(self.target)   # 原 bug：target_node 根本没定义！
        target_connected = False

        for _ in range(max_iter):
            rnd = self.generate_rnd()
            near_node = self.nearest_node(tree_nodes, rnd)

            # 与已有点坐标相同：跳过
            if rnd == near_node.coord:
                continue

            # 朝随机点走一步
            new_coord = self.steer(near_node.coord, rnd)

            # 自身无效：跳过
            if not self.is_valid_node(new_coord):
                continue

            # 先拿 near_node 当候选父，后面再用 rewire 优化 起点连线都撞了：直接丢弃
            if not self.is_valid_line(near_node.coord, new_coord):
                continue
            new_node = GridNode(new_coord)
            # ---- RRT* 重头戏：选最优父 + 重连邻居 ----
            rewired = self.rewire(tree_nodes, new_node)

            if not rewired:

                new_node = GridNode(new_coord)
                # 邻居圆为空 / 全撞了 → 走 RRT 兜底策略连最近邻 near_node
                new_node.parent = near_node
                new_node.g_cost = near_node.g_cost + self.distance(near_node.coord, new_node.coord)

            # new_node 正式上树
            tree_nodes.append(new_node)

            # 逐帧把当前树节点坐标抛给动图
            yield [tn.coord for tn in tree_nodes]

            # ---- 到达目标附近 & 无碰撞直连 → 收工 ----
            if (not target_connected
                    and self.distance(new_node.coord, self.target) <= self.STEP
                    and self.is_valid_line(new_node.coord, self.target)):
                target_connected = True
                target_node.parent = new_node
                target_node.g_cost = new_node.g_cost + self.distance(new_node.coord, self.target)
                tree_nodes.append(target_node)
                # 把"挂上目标"这一帧也 yield 出去（否则动图里看不到最后一条线）
                yield [tn.coord for tn in tree_nodes]
                return self.get_path(target_node), tree_nodes

        # 迭代耗尽也没到终点
        return None, tree_nodes

if __name__ == "__main__":

    rrt_star = RrtStar(robot_map=data.np_map, source=data.source, target=data.target)

    viz = dynamic.MapVisualizer()

    viz.plot_dynamic(
        lambda mi=5000: rrt_star.plan(mi),
        title="RRT* Path Planning Dynamic Result"
    )

