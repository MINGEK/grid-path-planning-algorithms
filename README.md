
# grid-path-planning-algorithms

使用 Python 实现的栅格地图路径规划算法
（Dijkstra、A*、Bidirectional-A*、JPS、RRT、RRT*、RRT-connect）。

## 特点

- **严格禁止切角**：斜向移动要求两侧正交栅格均可通行。
- 地图数据存储于可编辑的 Excel 文件中，由 30×30 矩形区域表示。
- 起点和终点在 `grid_map_data.py` 中配置。
- 代码包含类型注解和详细注释，记录设计思路与逻辑推导过程。
- 基于 Matplotlib 的动态可视化：算法以生成器（Generator）逐次 yield 搜索状态用于绘图，最终路径通过 `return` 返回；动画通过 `FuncAnimation` 的 `frames` 参数接收迭代器，并在生成器到达路径时抛出 `StopIteration` 异常，由 `try/except` 机制捕获后在最终帧绘制完整路径。

---
<details>
<summary>English</summary>

# grid-path-planning-algorithms

Python implementation of grid-based pathfinding algorithms 
（Dijkstra、A*、Bidirectional-A*、JPS、RRT、RRT*、RRT-connect）。

## Features

- **Strict no-corner-cutting**: diagonal movement requires both orthogonal cells to be free.
- Map data is stored in an editable Excel file as a 30x30 grid.
- Source and target are configured in `grid_map_data.py`.
- Code includes type annotations and detailed comments documenting the design rationale.
- Matplotlib dynamic visualization: the algorithm yields search states step by step via a generator for drawing, and returns the final path through `return`. The animation receives the iterator via `FuncAnimation`'s `frames` parameter; when the generator reaches the path it raises `StopIteration`, which is caught by a `try/except` block to render the complete path on the final frame.

</details>

## 项目结构
```
grid-path-planning-algorithms/
├── algorithms/
│   ├── Dijkstra.py
│   ├── A_star.py
│   ├── Bidirectional_A_star.py
│   ├── JPS.py
│   ├── RRT.py
│   ├── RRT_star.py
│   └── RRT_connect.py
├── utils/
│   ├── grid_map_data.py 
│   └── plt_dynamic.py
├── save_png/
│   ├── 动态绘图示例.gif
│   ├── map.png
│   ├── Dijkstra.png
│   ├── A_star.png
│   ├—─ Bidirectional_a_star.png
│   ├── JPS.png
│   ├── RRT.png
│   ├── RRT_star.png
│   └── RRT_connect.png
├── map.xlsx
├── requirements.txt
├── .gitignore
├── LICENSE
└── README.md
```
**Grid map**
<div align="center">
<img src="./save_png/map.png" alt="30×30栅格地图：黑色=障碍，白色=可通行" width="400"/>
</div>

**动态绘图示例**
<div align="center">
<img src="./save_png/动态绘图示例.gif" alt="动态绘图示例" />
</div>

**Dijkstra  Result**
<div align="center">
<img src="./save_png/Dijkstra.png" alt="Dijkstra Result Plt" />
</div>

**A\*  Result**
<div align="center">
<img src="./save_png/A_star.png" alt="A* Result Plt" />
</div>

**Bidirectional-A\* Result**
<div align="center">
<img src="./save_png/Bidirectional_A_star.png" alt="Bidirectional A* Result Plt" />
</div>

**JPS Result**
<div align="center">
<img src="./save_png/JPS.png" alt="JPS Result Plt" />
</div>

**RRT Result**
<div align="center">
<img src="./save_png/RRT.png" alt="RRT Result Plt" />
</div>

**RRT\* Result**
<div align="center">
<img src="./save_png/RRT_star.png" alt="RRT* Result Plt" />
</div>

**RRT-connect Result**
<div align="center">
<img src="./save_png/RRT_connect.png" alt="RRT_connect Result Plt" />
</div>