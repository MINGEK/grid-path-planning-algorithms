// ============================================================================
// astar_stepper.h — A* 步进器类声明
// ============================================================================
// 设计目的：把一次性跑完的 A* 算法拆成可单步执行的状态机
// 这样 raylib 每帧调用一次 step()，就能看到算法搜索过程动态展开
// ============================================================================

// 头文件保护宏（include guard）
// 目的：防止同一个头文件被重复包含导致重复定义
// #ifndef 如果没定义过这个宏，就继续编译；否则跳过到 #endif
// #define 定义这个宏；#endif 是保护区域的结束
#ifndef ASTAR_STEPPER_H    // 如果没有定义过 ASTAR_STEPPER_H
#define ASTAR_STEPPER_H    // 就定义它

// 引入需要的标准库头文件
#include <vector>          // std::vector：动态数组容器
#include <queue>           // std::priority_queue：优先队列（A* 的 Open 表）
#include <unordered_map>  // std::unordered_map：哈希表（存 g 值和父节点）
#include <utility>        // std::pair：存坐标对
#include <cmath>          // std::abs, std::sqrt：数学函数
#include <array>          // std::array：固定大小数组（存八个方向）

// ============================================================================
// AStarStepper 类
// ============================================================================
// class 关键字声明一个类，把数据和操作封装在一起
// 默认 private（私有），外部不能直接访问
class AStarStepper {

public:  // public 表示下面的成员可以被外部代码访问

    // ---- 嵌套枚举：格子状态 ----
    // enum 声明枚举类型，给一组整数值起名字
    // 目的：让绘制代码能查到每个格子当前属于哪个集合
    enum CellState {
        STATE_UNVISITED,  // 0：未访问（还没被算法发现）
        STATE_OPEN,       // 1：在 Open 表里（已发现，待扩展）
        STATE_CLOSED      // 2：在 Closed 表里（已扩展完毕）
    };

    // ---- 构造函数 ----
    // 与类同名的特殊成员函数，创建对象时自动调用
    // 参数用了 const&（只读引用），避免拷贝整个地图
    // 目的：初始化所有内部状态，把起点放进 Open 表
    AStarStepper(const std::vector<std::vector<int>>& grid,  // 地图引用（0=可通行 1=障碍）
                 int sourceRow, int sourceCol,                // 起点行列
                 int targetRow, int targetCol);               // 终点行列

    // ---- step()：执行一步 A* 迭代 ----
    // 从 Open 表弹出一个节点，扩展它的邻居
    // 返回 bool：true=算法仍在进行，false=算法结束
    // 目的：把原本 while 循环里的一轮拆出来，让外部控制节奏
    bool step();

    // ---- pathFound()：检查是否找到路径 ----
    // const 在函数签名末尾，表示这个函数不会修改成员变量
    // 目的：main.cpp 据此决定是否画金色路径
    bool pathFound() const { return found_; }  // 内联函数，直接返回成员变量

    // ---- isDone()：检查算法是否结束 ----
    // 目的：main.cpp 据此停止自动步进
    bool isDone() const { return done_; }

    // ---- getPath()：获取完整路径 ----
    // 返回 vector<pair<int,int>>，每个 pair 是一个格子的 {行, 列}
    // 目的：找到终点后顺着 parent 链回溯，返回有序坐标列表
    std::vector<std::pair<int, int>> getPath() const;

    // ---- getCellStates()：获取所有格子的当前状态 ----
    // 返回二维 vector，每个元素是 CellState 枚举
    // 目的：每帧绘制时调用，决定每个格子画什么颜色
    std::vector<std::vector<CellState>> getCellStates() const { return states_; }

    // ---- getCurrentNode()：获取当前正在扩展的节点 ----
    // 返回 pair<int,int>，{行, 列}
    // 目的：用橙色高亮标记算法当前在处理谁
    std::pair<int, int> getCurrentNode() const { return currentNode_; }

    // ---- getClosedCount()：获取已扩展节点数 ----
    // 目的：在屏幕上显示统计信息
    int getClosedCount() const { return closedCount_; }

    // ---- getOpenCount()：获取 Open 表当前大小 ----
    // static_cast<int> 把 priority_queue::size() 的 size_t 转 int
    // 目的：在屏幕上显示统计信息
    int getOpenCount() const { return static_cast<int>(openList_.size()); }

    // ---- getStepCount()：获取已执行步数 ----
    int getStepCount() const { return stepCount_; }

    // ---- 地图尺寸的 getter ----
    int getRows() const { return rows_; }  // 返回行数
    int getCols() const { return cols_; }  // 返回列数

private:  // private 表示下面的成员只有类内部能访问

    // ---- 嵌套结构体：A* 节点 ----
    // struct 和 class 类似，但默认 public
    // 目的：priority_queue 需要比大小来决定谁先出
    struct Node {
        int row, col;        // 格子的行列坐标
        double f, g, h;      // f=总代价估计, g=起点到此的实际代价, h=到此到终点的估计

        // 重载大于号运算符（operator>）
        // 目的：让 priority_queue 配 greater<Node> 变成小顶堆（f 小的先出）
        // 原因：priority_queue 默认大顶堆（用 less），用 greater 反过来
        // 参数加 const& 避免拷贝，函数末尾加 const 表示不修改 this
        bool operator>(const Node& other) const {
            if (f == other.f) return g > other.g;  // f 相同时 g 大的优先级低
            return f > other.f;                     // 否则 f 大的优先级低
        }
    };

    // ---- 嵌套结构体：移动方向 ----
    struct Dir {
        int dRow, dCol;     // 行列偏移量（比如 -1,0 表示往上走一行）
        double cost;        // 移动代价：直行=1.0，斜行=sqrt(2)≈1.414
    };

    // ---- 内部数据成员 ----

    // 地图引用：用引用而不是拷贝，省内存
    // 注意：引用必须在构造函数初始化列表里绑定，之后不能改绑
    const std::vector<std::vector<int>>& grid_;  // 地图数据（0=可通行 1=障碍）

    int rows_;           // 地图行数
    int cols_;           // 地图列数
    int sourceRow_;      // 起点行号
    int sourceCol_;      // 起点列号
    int targetRow_;      // 终点行号
    int targetCol_;      // 终点列号

    // Open 表：优先队列（小顶堆），f 最小的在堆顶
    // 模板参数：<元素类型, 底层容器, 比较器>
    // greater<Node> 让最小的在堆顶（默认 less<Node> 是最大的在堆顶）
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList_;

    // g 值表：哈希表，key=格子编号，value=从起点到这个格子的最小已知代价
    // 目的：判断新路径是否更优，避免重复扩展已关闭的节点
    std::unordered_map<int, double> gScore_;

    // 父节点表：哈希表，key=格子编号，value=父格子的编号
    // 目的：到达终点后顺着它回溯出完整路径
    std::unordered_map<int, int> parent_;

    // 状态表：二维 vector，记录每个格子是 UNVISITED/OPEN/CLOSED
    // 目的：绘制时查询，也是算法状态的直观反映
    std::vector<std::vector<CellState>> states_;

    std::pair<int, int> currentNode_;  // 当前正在扩展的节点坐标 {行, 列}
    bool done_;                        // 算法是否结束（找到路径或无解）
    bool found_;                       // 是否找到了路径
    int closedCount_;                  // 已扩展节点数（Closed 集合大小）
    int stepCount_;                    // 已执行步数

    // ---- 私有辅助方法 ----

    // 把行列坐标编码成唯一整数 key
    // 目的：unordered_map 用 int 做 key 比 pair 快
    // 公式：row * cols + col，保证不同坐标映射到不同整数
    int key(int row, int col) const { return row * cols_ + col; }

    // 曼哈顿距离启发函数
    // 目的：估计当前格子到终点的代价，指导搜索朝终点方向走
    // 原理：|行差| + |列差|，在网格地图上是 admissible（不高估）的
    double heuristic(int r1, int c1, int r2, int c2) const {
        return std::abs(r1 - r2) + std::abs(c1 - c2);  // 行差绝对值 + 列差绝对值
    }

    // 获取八方向常量数组
    // static 表示属于类而不是某个对象
    // 目的：返回一个只初始化一次的静态数组，避免重复构造
    static const std::array<Dir, 8>& getDirs();
};

#endif  // 结束头文件保护宏
