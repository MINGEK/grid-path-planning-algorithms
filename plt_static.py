import grid_map_data as data
import numpy as np                                                            # 数值计算库，用于矩阵运算和数组操作
import matplotlib.pyplot as plt                                               # 绑图库，用于可视化栅格地图和路径
                                                             # 数学库，用于 sqrt 等数学函数

"""坐标x，y表示第x行，第y列"""
def map_visualization(grid_map = data.np_map, path=None, title="Path Planning", explored=None):           # 定义绑图函数：map=栅格地图, path=路径节点列表, title=标题, explored=已探索节点集合
    config = {                                                                 # matplotlib 全局配置字典
        "font.family": 'serif',                                               # 字体族：衬线字体
        "font.serif": ['Times New Roman', 'SimSun'],                         # 优先使用宋体，回退到 Times New Roman
        "font.size": 12,                                                       # 全局字体大小
        "axes.unicode_minus": False,                                          # 解决负号显示为方块的问题
        'figure.dpi': 150,                                                     # 显示分辨率（每英寸像素数）
        'savefig.dpi': 300,                                                     # 保存图片分辨率
    }
    plt.rcParams.update(config)                                                # 应用全局配置到 matplotlib

    # 获取地图尺寸
    height, width = grid_map.shape  # height=30, width=30                           # 解构地图形状：行数=高度=30，列数=宽度=30

    # 创建一个新的图形窗口
    fig, ax = plt.subplots(figsize=(8, 8))                                  # 创建 10x10 英寸的画布和坐标轴对象

    # 使用imshow绘制网格地图，障碍物显示为黑色
    ax.imshow(grid_map,  # 绘制栅格地图数据
              cmap='Greys',  # 灰度色系 0=白色(自由), 1=黑色(障碍)
              origin='upper',  # 设定坐标系原点位置为左上角
              # 默认图形边界从-0.5开始，
              # extent=(-0.5, width-0.5, height-0.5, -0.5),
              alpha=0.6,  # 透明度 0.6                        # 地图透明度设为0.6，使叠加的路径和标记更清晰
              vmin=0,  # 颜色映射最小值：0 映射为纯白
              vmax=1,  # 颜色映射最大值：1 映射为纯黑
              interpolation='none',  # 像素硬边界
              aspect='equal')                 # x, y 单位长度等长，方格

    # 设置主刻度位置 格子中心
    ax.set_xticks(np.arange(0, width, 1))
    ax.set_yticks(np.arange(0, height, 1))

    # 设置主刻度标签
    ax.set_xticklabels(list(map(str, range(width))))
    ax.set_yticklabels(list(map(str, range(height))))

    # 隐藏双轴次要刻度线（只显示主要刻度） 次要刻度仅用于网格线定位
    ax.tick_params(axis='both', which='minor', length=0)

    # 设置网格线在格子边界上（使用次要刻度）
    ax.set_xticks(np.arange(-0.5, width,  1), minor=True )                     # 设置 x 轴次要刻度在格子边界：0, 1, 2, ..., 30
    ax.set_yticks(np.arange(-0.5, height, 1), minor=True )                     # 设置 y 轴次要刻度在格子边界：0, 1, 2, ..., 30
    ax.grid(True, which='minor', alpha=0.5, linestyle='-', linewidth=1.5 )   # 在次要刻度位置绘制网格线：透明度0.5, 实线, 线宽1.5

    # 四处轴刻度 各留出半格，以视图居中，为美观(原默认尺寸为[-0.5, width-0.5],单格尺寸为1,再扩展半格尺寸为[-1,width])
    ax.set_xlim(-1, width)
    ax.set_ylim(height, -1)

    # 调整轴标签位置
    ax.xaxis.set_label_coords(0.5, -0.05)  # X轴标签向下移动                  # 将 x 轴标签位置调整到画布水平中央、底部偏下
    ax.yaxis.set_label_coords(-0.06, 0.5)  # Y轴标签向左移动                  # 将 y 轴标签位置调整到画布左侧偏外、垂直中央

    # 增加刻度标签与轴的距离
    ax.tick_params(axis='x', pad=6, labelsize=9)                                           # x 轴刻度标签与轴的距离增加8磅，防止与网格重叠
    ax.tick_params(axis='y', pad=6, labelsize=9)                                           # y 轴刻度标签与轴的距离增加8磅，防止与网格重叠

    # 用蓝色星号标记起点位置（格子中心）
    ax.scatter(data.source[1], data.source[0],
               c='blue', s=80, marker='o', zorder=10, label='Start')          # 颜色蓝色, 大小80, 圆形标记, 层级10（最上层）, 图例'Start'
    # 用红色星号标记终点位置（格子中心）
    ax.scatter(data.target[1], data.target[0],
               c='m', s=80, marker='s', zorder=10, label='Goal')              # 颜色品红, 大小80, 方形标记, 层级10, 图例'Goal'

    # 如果提供了已探索节点集合
    if explored is not None and len(explored) > 0:                             # 检查是否传入已探索节点集合并非空
        # 坐标移到格子中心
        explored_arr = np.array([[col, row] for row, col in explored])
        # 用浅蓝色半透明标记绘制已探索区域
        ax.scatter(explored_arr[:, 0], explored_arr[:, 1],                    # 绑制已探索节点的散点图，x=第0列, y=第1列
                   c='lightblue', s=10, alpha=0.5, label='Explored')          # 颜色浅蓝, 点大小10, 透明度0.5, 图例标签'Explored'

    # 如果提供了路径
    if path is not None and len(path) > 0:                                     # 检查是否传入路径节点列表面非空
        # 坐标移到格子中心
        path_arr = np.array([[col, row] for row, col in path])                                 # 转为 numpy 数组
        # 用红色线条绘制路径
        ax.plot(path_arr[:, 0], path_arr[:, 1], 'r-', linewidth=2, label='Path')  # 绑制红色实线路径线，线宽2
        # 用绿色圆点标记路径上的每个节点
        ax.scatter(path_arr[:, 0], path_arr[:, 1],                            # 在路径每个节点上绑制散点
                   c='lime', s=30, zorder=5, edgecolors='darkgreen')          # 颜色青绿, 大小30, 层级5, 深绿色边框


    # 设置图表标题
    ax.set_title(title, fontsize=14)                                          # 设置标题，字体大小14
    # 设置x轴标签
    ax.set_xlabel('col', fontsize=12)                                          # x 轴标签为 'width'（列），字体大小12
    # 设置y轴标签
    ax.set_ylabel('row', fontsize=12)                                         # y 轴标签为 'height'（行），字体大小12
    # 调整布局，防止标签被截断
    plt.tight_layout()                                                        # 自动调整子图参数，防止标签和标题被截断
    # 显示图形
    plt.show()                                                                # 显示图形窗口（阻塞式调用，关闭窗口后程序继续）

if __name__ == "__main__":                                                    # 当脚本直接运行时执行（而非被导入时）
    map_visualization()                                                          # 调用绑图函数，仅显示栅格地图（不传 path 和 explored 参数）