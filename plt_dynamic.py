"""
地图可视化模块
提供静态地图绘制和动态路径规划展示功能
"""
import grid_map_data as data
import numpy as np  # 导入numpy库，用于数值计算
import matplotlib.pyplot as plt  # 导入matplotlib绘图库
from matplotlib.animation import FuncAnimation  # 导入动画模块
from typing import Optional, List, Tuple, Set, Callable  # 导入类型注解


class MapVisualizer:
    """地图可视化器，提供静态和动态绘图功能"""

    def __init__(self, grid_map: np.ndarray = data.np_map,
                 source: Tuple[int, int] = data.source,
                 target: Tuple[int, int] = data.target) -> None:
        """
        初始化可视化器

        Args:
            grid_map: 栅格地图二维数组，0=自由空间，1=障碍物
            source: 起点坐标 (row, col)
            target: 终点坐标 (row, col)
        """
        self.grid_map = grid_map  # 保存栅格地图数据
        self.source = source  # 保存起点坐标
        self.target = target  # 保存终点坐标
        self.height, self.width = grid_map.shape  # 获取地图尺寸

        # 配置matplotlib全局参数
        self._configure_matplotlib()  # 调用配置方法

    def _configure_matplotlib(self) -> None:
        """配置matplotlib全局参数"""
        config = {  # 配置字典
            "font.family": 'serif',  # 字体族
            "font.serif": ['Times New Roman', 'SimSun'],
            "font.size": 12,  # 字体大小
            "axes.unicode_minus": False,  # 解决负号显示问题
            'figure.dpi': 150,  # 显示分辨率
            'savefig.dpi': 300,  # 保存分辨率
        }
        plt.rcParams.update(config)  # 应用配置

    def _setup_figure(self) -> Tuple[plt.Figure, plt.Axes]:
        """创建图形和坐标轴"""
        fig, ax = plt.subplots(figsize=(8, 8))  # 创建画布
        return fig, ax  # 返回图形和坐标轴

    def _draw_base_map(self, ax: plt.Axes) -> None:
        """绘制基础地图层"""
        # 绘制栅格地图
        ax.imshow(self.grid_map,  # 绘制栅格地图数据
                  cmap='Greys',  # 灰度色系 0=白色(自由), 1=黑色(障碍)
                  origin='upper',  # 设定坐标系原点位置为左上角
                  # 默认图形边界从-0.5开始，
                  # extent=(-0.5, width-0.5, height-0.5, -0.5),
                  alpha=0.6,  # 透明度 0.6                        # 地图透明度设为0.6，使叠加的路径和标记更清晰
                  vmin=0, vmax=1, # 颜色映射最小值：0 映射为纯白,1 映射为纯黑
                  interpolation='none',  # 像素硬边界
                  aspect='equal')  # x, y 单位长度等长，方格

        # 设置主刻度在格子中心
        ax.set_xticks(np.arange(0, self.width, 5))
        ax.set_yticks(np.arange(0, self.height, 5))
        ax.set_xticklabels([str(i) for i in range(0, self.width, 5)])
        ax.set_yticklabels([str(i) for i in range(0, self.height, 5)])

        # 设置次刻度在格子边界
        ax.set_xticks(np.arange(-0.5, self.width, 1), minor=True)  # x轴次刻度
        ax.set_yticks(np.arange(-0.5, self.height, 1), minor=True)  # y轴次刻度

        # 隐藏次刻度线
        ax.tick_params(axis='both', which='minor', length=0)  # 次刻度线长度0

        # 绘制网格线
        ax.grid(True, which='minor', alpha=0.5, linestyle='-', linewidth=1.5)  # 网格线

        # 设置坐标轴范围
        ax.set_xlim(-1, self.width)  # x轴范围
        ax.set_ylim(self.height, -1)  # y轴范围

        # 设置刻度标签样式
        ax.tick_params(axis='both', labelsize=9, pad=4)  # 标签字号和距离

        # 设置轴标签
        ax.set_xlabel('col', fontsize=12, )  # x轴标签
        ax.set_ylabel('row', fontsize=12)  # y轴标签

        # 绘制起点
        ax.scatter(self.source[1], self.source[0], c='blue', s=80, marker='o', zorder=11, label='Start')

        # 绘制终点
        ax.scatter(self.target[1], self.target[0], c='magenta', s=80, marker='s', zorder=11, label='Goal')

    def plot_static(self,
                    path: Optional[List[Tuple[int, int]]] = None,
                    explored: Optional[Set[Tuple[int, int]]] = None,
                    title: str = "Static Map") -> None:
        """
        绘制静态地图

        Args:
            path: 路径节点列表
            explored: 已探索节点集合
            title: 图表标题
        """
        fig, ax = self._setup_figure()  # 创建图形
        self._draw_base_map(ax)  # 绘制基础地图

        # 绘制已探索区域
        if explored:
            explored_arr = np.array([[col, row] for row, col in explored])  # 转换坐标
            ax.scatter(explored_arr[:, 0], explored_arr[:, 1],  # 散点图
                       c='lightblue', s=10, alpha=0.5,  # 样式
                       label='Explored')  # 图例

        # 绘制路径
        if path:
            path_arr = np.array([[col, row] for row, col in path])
            ax.plot(path_arr[:, 0], path_arr[:, 1],  # 路径线
                    'r-', linewidth=2, label='Path')  # 红色实线
            ax.scatter(path_arr[:, 0], path_arr[:, 1],  # 路径节点
                       c='lime', s=30, zorder=5,  # 青绿色
                       edgecolors='darkgreen')  # 深绿边框

        ax.set_title(title, fontsize=14)  # 设置标题
        plt.tight_layout()  # 调整布局
        plt.show()  # 显示图形

    def plot_dynamic(self,
                     path_plan_func: Callable = None,
                     title: str = "Dynamic Map") -> None:
        """
        绘制动态地图 实时更新
        """
        if path_plan_func:
            gen = path_plan_func()
        else:
            return None
        #创建图、轴对象
        fig, ax = self._setup_figure()
        #绘制地图底图
        self._draw_base_map(ax)

        # 创建动态对象
        explored_points = ax.scatter([], [], c='#4a90e2', s=30, alpha=0.28, label='explored')
        path_points = ax.scatter([], [], c='lime', edgecolors='darkgreen',s=35,zorder=10)      # 颜色青绿, 大小30, 层级5, 深绿色边框
        path_line, = ax.plot([], [],'r-', linewidth=2.5, label='Path',zorder = 5)

        def init():
            """动画初始化"""
            path_line.set_data([], [])
            explored_points.set_offsets(np.empty((0, 2)))
            path_points.set_offsets(np.empty((0, 2)))
            return [explored_points, path_line, path_points]

        def update(frame):
            try:
                gen_points = next(gen)
            except StopIteration as e:
                path, explored_all = e.value
                if path:
                    # 绘制路径
                    path_arr = np.array([[col, row] for row, col in path])
                    path_line.set_data(path_arr[:, 0], path_arr[:, 1])
                    path_points.set_offsets(path_arr)
                self.ani.event_source.stop()
                return [explored_points, path_line, path_points]

            if gen_points:
                points = [[col, row] for row, col in gen_points]
                explored_points.set_offsets(points)
            return [explored_points, path_line, path_points]

        # 创建动画
        self.ani = FuncAnimation(
            fig,
            func=update,
            frames=None,
            init_func=init,
            interval=2,  # 建议调慢一点便于观察
            blit=False,
            cache_frame_data=False  # 添加这个避免警告1
        )

        plt.title(title, fontsize=14)
        plt.tight_layout()
        plt.show()

def test():
    explor = [[(2,3),(3,4)],[(4,3),(4,4)]]
    for i in explor:
        yield i
    return [(2,3),(3,4),(4,3),(4,4)],{(2,3)}

if __name__ == '__main__':
    viz = MapVisualizer()                    # 先创建实例
    #viz.plot_static()         # 再调用方法
    viz.plot_dynamic(test)