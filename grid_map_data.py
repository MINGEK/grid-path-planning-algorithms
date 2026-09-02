import pandas as pd                                                           # 数据处理库，用于读取 Excel 文件
import math
# 我将栅格图以二元值的形式存入到了excel中，在后面的修改我我可以直观的修改图的内容
# 栅格地图是30*30 不跳过行，读取30行数据（不包括首行，首行作为标题行），读取第2-31列，
df_map = pd.read_excel('map.xlsx', usecols=list(range(1, 31)), skiprows=0, nrows=30, header=0)  # 从 Excel 读取栅格地图：usecols 指定读取第2~31列（索引1~30），skiprows=0 不跳过行，nrows=30 读取30行，header=0 将第一行作为列名
# 转化为np数组
np_map = df_map.to_numpy()                                                    # 将 DataFrame 转为 numpy 二维数组，0=自由空间，1=障碍物
# 创建八邻域搜素
# (d_rows,d_cols,distance)
d_8 = [                                                                       # 定义八邻域偏移列表，每个元素为 (行偏移, 列偏移, 移动距离)
    (-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),              # 上左(对角线,距离√2), 上(直线,距离1), 上右(对角线,距离√2)
    (0, -1, 1.0),                          (0, 1, 1.0),                       # 左(直线,距离1),                          右(直线,距离1)
    (1, -1, math.sqrt(2)),  (1, 0, 1.0),  (1, 1, math.sqrt(2)),               # 下左(对角线,距离√2), 下(直线,距离1), 下右(对角线,距离√2)
]
# 创建起点、终点
source = (28, 4)                                                              # 起点：(第28行, 第4列)，坐标格式为 (row, col)
target = (6, 24)                                                               # 终点：(第6行, 第24列)，坐标格式为 (row, col)