// ============================================================================
// astar_stepper.cpp — A* 步进器类实现
// ============================================================================
// 实现 astar_stepper.h 里声明的所有方法
// 核心思想：把 A* 算法的 while 循环拆成 step() 函数，每次只执行一轮
// ============================================================================

#include "astar_stepper.h"  // 引入对应的头文件（类声明）
#include <algorithm>        // std::reverse：反转路径顺序

// ============================================================================
// 获取八方向常量数组
// ============================================================================
// static 成员函数不需要通过对象调用，可以直接用 AStarStepper::getDirs()
// 返回 const 引用，避免拷贝
// 目的：返回一个只初始化一次的静态数组，描述八个移动方向
const std::array<AStarStepper::Dir, 8>& AStarStepper::getDirs() {

    // static 局部变量：只在第一次调用时构造，之后直接返回
    // 目的：避免每次调用 step() 都重新构造数组
    static const std::array<Dir, 8> dirs = {{  // {{ }} 双花括号：array 的聚合初始化
        // 八个方向：{行偏移, 列偏移, 代价}
        // 直行代价 = 1.0，斜行代价 = sqrt(2) ≈ 1.414
        {-1, -1, std::sqrt(2.0)},  // 左上：行-1 列-1，斜行
        {-1,  0, 1.0},             // 正上：行-1 列 0，直行
        {-1,  1, std::sqrt(2.0)},  // 右上：行-1 列+1，斜行
        { 0, -1, 1.0},             // 正左：行 0 列-1，直行
        { 0,  1, 1.0},             // 正右：行 0 列+1，直行
        { 1, -1, std::sqrt(2.0)},  // 左下：行+1 列-1，斜行
        { 1,  0, 1.0},             // 正下：行+1 列 0，直行
        { 1,  1, std::sqrt(2.0)}   // 右下：行+1 列+1，斜行
    }};
    return dirs;  // 返回这个静态数组的引用
}

// ============================================================================
// 构造函数：初始化所有内部状态
// ============================================================================
// 函数名 AStarStepper::AStarStepper：类名::类名 是构造函数
// 参数列表后面跟着的 : 开始的是"成员初始化列表"
// 成员必须在初始化列表里初始化的情况：引用、const 成员、没有默认构造函数的成员
AStarStepper::AStarStepper(const std::vector<std::vector<int>>& grid,
                           int sourceRow, int sourceCol,
                           int targetRow, int targetCol)

    // 成员初始化列表：在构造函数体执行前初始化各成员
    : grid_(grid),                     // 绑定地图引用（引用必须在初始化列表绑定）
      rows_(static_cast<int>(grid.size())),         // 行数 = grid 的元素个数
      cols_(static_cast<int>(grid[0].size())),      // 列数 = 第一行的元素个数
      sourceRow_(sourceRow),           // 起点行
      sourceCol_(sourceCol),           // 起点列
      targetRow_(targetRow),           // 终点行
      targetCol_(targetCol),           // 终点列
      currentNode_({sourceRow, sourceCol}),  // 当前节点初始化为起点
      done_(false),                    // 算法未结束
      found_(false),                   // 还没找到路径
      closedCount_(0),                 // 已扩展数为 0
      stepCount_(0) {                  // 步数为 0

    // ---- 初始化状态表 ----
    // states_.assign(n, value)：把 states_ 重置为 n 个 value
    // 这里初始化为 rows_ 行 cols_ 列，每个格子都是 STATE_UNVISITED
    // 内层 vector<CellState> 给每个值赋 STATE_UNVISITED
    states_.assign(rows_, std::vector<CellState>(cols_, STATE_UNVISITED));

    // ---- 创建起点节点 ----
    Node source;              // 声明一个 Node 结构体变量
    source.row = sourceRow_;  // 起点的行号
    source.col = sourceCol_;  // 起点的列号
    source.g = 0;             // 起点到自身的实际代价是 0
    source.h = heuristic(sourceRow_, sourceCol_, targetRow_, targetCol_);  // 启发值
    source.f = source.g + source.h;  // f = g + h（总代价估计）

    // ---- 把起点放进 Open 表 ----
    // .push() 把元素压入优先队列
    openList_.push(source);

    // ---- 记录起点的 g 值 ----
    // 目的：后续扩展邻居时和它比较，判断新路径是否更优
    // gScore_[key] = value：哈希表插入/更新
    gScore_[key(sourceRow_, sourceCol_)] = 0;

    // ---- 起点标记为 Open 状态 ----
    // 目的：绘制时显示蓝色（已在 Open 表待扩展）
    states_[sourceRow_][sourceCol_] = STATE_OPEN;
}

// ============================================================================
// step()：执行一步 A* 迭代
// ============================================================================
// 返回 bool：true=算法仍在进行，false=算法结束
bool AStarStepper::step() {

    // 如果算法已经结束，直接返回 false
    if (done_) return false;

    // 如果 Open 表空了，说明所有可达节点都扩展完了，无解
    if (openList_.empty()) {
        done_ = true;     // 标记算法结束
        found_ = false;   // 没找到路径
        return false;
    }

    // ---- 从 Open 表取出 f 最小的节点 ----
    // .top() 访问堆顶元素（f 最小的），但不删除
    Node current = openList_.top();  // 拷贝堆顶节点
    // .pop() 删除堆顶元素（注意 pop 不返回值，所以要先 top 再 pop）
    openList_.pop();

    // ---- 延迟删除：跳过过时节点 ----
    // priority_queue 不支持更新优先级，所以同一个格子可能被多次入队
    // 如果当前节点的 g 值比 gScore_ 里记录的大，说明它已被更优路径访问过
    // while 循环：持续弹栈直到找到有效的节点或队列空
    while (!openList_.empty() && current.g > gScore_[key(current.row, current.col)]) {
        current = openList_.top();  // 取下一个堆顶
        openList_.pop();            // 弹出
    }

    // 再检查一次（上面的 while 可能因为队列空而退出）
    if (current.g > gScore_[key(current.row, current.col)]) {
        // 这种情况下当前节点也是过时的，且队列已空
        done_ = true;
        found_ = false;
        return false;
    }

    // ---- 记录当前正在扩展的节点 ----
    // 目的：main.cpp 用橙色高亮标记它
    currentNode_ = {current.row, current.col};  // {行, 列}

    // ---- 把当前节点标记为 Closed ----
    // 目的：绘制时显示灰色，后续不再重复扩展
    states_[current.row][current.col] = STATE_CLOSED;
    closedCount_++;   // 已扩展计数+1
    stepCount_++;      // 步数+1

    // ---- 检查是否到达终点 ----
    if (current.row == targetRow_ && current.col == targetCol_) {
        done_ = true;    // 算法结束
        found_ = true;  // 找到了
        return false;   // 返回 false 表示算法不再继续
    }

    // ---- 扩展当前节点的八个邻居 ----
    // range-based for（C++11）：遍历 getDirs() 返回的数组
    // const auto& dir：auto 自动推导为 Dir 类型，& 避免拷贝
    for (const auto& dir : getDirs()) {

        int nr = current.row + dir.dRow;  // 邻居行 = 当前行 + 方向偏移
        int nc = current.col + dir.dCol;  // 邻居列 = 当前列 + 方向偏移

        // ---- 越界检查 ----
        // 如果邻居在地图范围外，跳过
        // || 是逻辑或，只要有一个条件为 true 就 continue
        if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;

        // ---- 障碍检查 ----
        // 如果邻居是障碍物（值为 1），跳过
        if (grid_[nr][nc] == 1) continue;

        // ---- 斜行不能穿越障碍 ----
        // 目的：防止路径"贴着障碍角"穿越，物理上不合理
        // 原理：斜行时，水平方向和垂直方向的相邻格子必须都可通行
        // dir.cost > 1.0 判断是否是斜行（直行=1.0，斜行=sqrt(2)≈1.414）
        if (dir.cost > 1.0) {

            int adj1Row = current.row;   // 水平邻居的行（和当前节点相同）
            int adj1Col = nc;            // 水平邻居的列（和邻居相同）
            int adj2Row = nr;            // 垂直邻居的行（和邻居相同）
            int adj2Col = current.col;   // 垂直邻居的列（和当前节点相同）

            // 如果两个相邻格子有一个是障碍，就不能斜行
            if (grid_[adj1Row][adj1Col] == 1 || grid_[adj2Row][adj2Col] == 1) {
                continue;  // 跳过这个方向
            }
        }

        // ---- 计算从起点经过当前节点到邻居的代价 ----
        // tentativeG = 当前节点的 g + 这一步的移动代价
        double tentativeG = current.g + dir.cost;

        // 计算邻居的唯一 key（用于查哈希表）
        int nk = key(nr, nc);

        // ---- 判断是否需要更新这个邻居 ----
        // gScore_.find(nk) 在哈希表里查找 key=nk
        // .end() 表示没找到（指向末尾迭代器）
        // 如果邻居没访问过（find==end），或新路径更优（tentativeG < 已有值）
        if (gScore_.find(nk) == gScore_.end() || tentativeG < gScore_[nk]) {

            gScore_[nk] = tentativeG;                    // 更新 g 值
            parent_[nk] = key(current.row, current.col); // 记录父节点

            // 创建邻居节点并入队
            Node neighbor;                       // 声明 Node 变量
            neighbor.row = nr;                   // 邻居行
            neighbor.col = nc;                   // 邻居列
            neighbor.g = tentativeG;             // 新的 g 值
            neighbor.h = heuristic(nr, nc, targetRow_, targetCol_);  // 启发值
            neighbor.f = neighbor.g + neighbor.h;                     // f = g + h
            openList_.push(neighbor);           // 压入 Open 表

            // 标记为 Open 状态
            // 目的：绘制时显示蓝色
            states_[nr][nc] = STATE_OPEN;
        }
    }

    return true;  // 算法仍在进行
}

// ============================================================================
// getPath()：获取从起点到终点的完整路径
// ============================================================================
// 返回 vector<pair<int,int>>：路径坐标列表，起点在前终点在后
// const 在函数末尾：不修改成员变量
std::vector<std::pair<int, int>> AStarStepper::getPath() const {

    // 声明返回的路径变量
    std::vector<std::pair<int, int>> path;

    // 如果没找到路径，返回空 vector
    if (!found_) return path;

    // ---- 从终点开始回溯 ----
    int curRow = targetRow_;  // 当前回溯位置：终点行
    int curCol = targetCol_;  // 当前回溯位置：终点列

    // ---- 顺着 parent 链一直回溯到起点 ----
    // while 循环：条件是"当前不是起点"
    // && 是逻辑与，两个条件都不满足时才继续回溯
    while (!(curRow == sourceRow_ && curCol == sourceCol_)) {

        // emplace_back 比 push_back 更高效：直接在 vector 末尾构造元素
        // 把当前节点加入路径
        path.emplace_back(curRow, curCol);

        // 从哈希表查父节点
        int k = key(curRow, curCol);  // 当前节点的 key
        int parentK = parent_.at(k);  // .at() 比 [] 更安全（越界会抛异常）

        // 从 key 反解出父节点的行列
        // key = row * cols + col，所以 row = key / cols，col = key % cols
        curRow = parentK / cols_;     // 整数除法得到父节点行号
        curCol = parentK % cols_;     // 取模运算得到父节点列号
    }

    // 把起点加入路径
    path.emplace_back(sourceRow_, sourceCol_);

    // ---- 反转路径 ----
    // 回溯出来的顺序是：终点 → ... → 起点（反着的）
    // 反转后变成：起点 → ... → 终点（符合阅读习惯）
    // std::reverse 是原地反转（修改原容器），不返回新容器
    std::reverse(path.begin(), path.end());

    return path;  // 返回路径（编译器会做 RVO 优化，不产生拷贝）
}
