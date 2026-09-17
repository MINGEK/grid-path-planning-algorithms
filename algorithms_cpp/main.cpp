// ============================================================================
// A* 路径规划 raylib 动态可视化主程序
// ============================================================================
// 功能：步进式展示 A* 算法在网格地图上的搜索过程
// 地图格式：CSV 文件，0=可通行，1=障碍
// 交互方式：空格=单步执行，回车=连续自动，R=重置，ESC=退出
// ============================================================================

// #include 是预处理指令，在编译前把指定头文件的内容原封不动地粘贴到这个位置
// "raylib.h" 用双引号：先在当前目录找，再去系统目录找
// <iostream> 用尖括号：只去系统目录找（标准库和第三方库）

#include "raylib.h"        // raylib 图形库的主头文件，声明了 InitWindow、DrawRectangle 等所有图形 API
#include "csv_reader.h"    // 你自己写的 CSV 读取器类，声明了 readRegion 方法
#include "astar_stepper.h" // A* 步进器类，把一次性算法拆成可单步执行的状态机

#include <vector>   // C++ 标准库的动态数组容器 std::vector
#include <string>   // C++ 标准库的字符串类 std::string
#include <memory>   // C++ 智能指针 std::unique_ptr，自动管理内存

// ============================================================================
// 全局配置常量
// ============================================================================
// const 修饰的变量不可修改，编译器会把它放到只读区域
// 全局 const 变量在整个文件里都能访问，适合放配置参数

// 地图文件路径（相对路径，相对于程序运行时的当前目录）
// 目的：指定要读取的 CSV 地图文件名
const char* MAP_FILE = "map.csv";  // const char* 是指向常量字符串的指针，C 风格字符串

// 起点坐标：第 28 行第 4 列（基于 readRegion 跳过表头后的 30x30 地图）
// 目的：固定演示路径的起点位置，方便对比算法行为
const int SOURCE_ROW = 28;  // 行号，从 0 开始计数
const int SOURCE_COL = 4;   // 列号，从 0 开始计数

// 终点坐标：第 6 行第 24 列
// 目的：固定演示路径的终点位置
const int TARGET_ROW = 6;   // 终点行号
const int TARGET_COL = 24;  // 终点列号

// 窗口尺寸（像素）
// 目的：设置窗口大小，1200x800 让 30x30 地图每个格子足够大
const int WINDOW_W = 1200;  // 窗口宽度（像素）
const int WINDOW_H = 800;   // 窗口高度（像素）

// 网格绘制区域的左上角偏移量
// 目的：给顶部信息栏（70px）和左右边距留出空间，网格不贴着窗口边缘
const int GRID_OFFSET_X = 50;  // 网格左边距：距窗口左边缘 50 像素
const int GRID_OFFSET_Y = 80;  // 网格上边距：距窗口顶部 80 像素（留给信息栏）

// ============================================================================
// 颜色定义
// ============================================================================
// raylib 的 Color 类型是 { r, g, b, a } 四个 unsigned char（0~255）
// r=红, g=绿, b=蓝, a=透明度（255=完全不透明）
// 目的：用不同颜色区分各种格子状态，直观展示算法过程

const Color COLOR_UNVISITED = { 245, 245, 245, 255 };  // 浅灰白：还没被算法访问到的格子
const Color COLOR_OBSTACLE   = { 30, 30, 30, 255 };   // 深黑：障碍物，不可通行
const Color COLOR_OPEN       = { 70, 130, 220, 255 };  // 蓝色：在 Open 表里（已发现，待扩展）
const Color COLOR_CLOSED     = { 170, 170, 180, 255 }; // 灰色：在 Closed 表里（已扩展完毕）
const Color COLOR_CURRENT    = { 255, 165, 0, 255 };   // 橙色：当前正在扩展的节点（高亮描边）
const Color COLOR_PATH       = { 255, 200, 50, 255 };  // 金色：最终找到的路径
const Color COLOR_START      = { 60, 200, 100, 255 };  // 绿色：起点
const Color COLOR_GOAL       = { 220, 60, 60, 255 };   // 红色：终点
const Color COLOR_GRID_LINE  = { 200, 200, 200, 255 }; // 浅灰：网格线，让格子边界清晰
const Color COLOR_BG         = { 250, 250, 252, 255 };  // 近白：窗口背景色

// ============================================================================
// 辅助函数：计算格子尺寸
// ============================================================================
// 函数签名：void 表示无返回值
// 参数用了引用传递（&），可以直接修改调用方的变量
// 目的：根据地图行列数和窗口可用空间，自动计算每个格子的像素边长
void calcCellSize(int rows, int cols, int& cellSize, int& gridPixelW, int& gridPixelH) {

    // 计算可用宽度：窗口宽 - 左右各留一个偏移
    int availW = WINDOW_W - 2 * GRID_OFFSET_X;  // 减去左右边距，得到网格可用宽度

    // 计算可用高度：窗口高 - 上边距 - 底部留 30px 给图例
    int availH = WINDOW_H - GRID_OFFSET_Y - 30;  // 减去顶部信息栏和底部图例区域

    // 按宽度算每个格子的尺寸：可用宽 / 列数
    int sizeByW = availW / cols;  // 如果只受宽度限制，每个格子的边长

    // 按高度算每个格子的尺寸：可用高 / 行数
    int sizeByH = availH / rows;  // 如果只受高度限制，每个格子的边长

    // 取较小的值，保证格子是正方形且不超出可用区域
    // 三目运算符：条件 ? 真值 : 假值
    cellSize = (sizeByW < sizeByH) ? sizeByW : sizeByH;  // 取宽高方向受限更严格的那个

    // 下限保护：格子太小看不清，至少 8 像素
    if (cellSize < 8) cellSize = 8;  // if 语句：如果条件成立，执行赋值

    // 计算网格总像素尺寸：格子大小 * 格子数
    gridPixelW = cellSize * cols;  // 网格总宽度（像素）
    gridPixelH = cellSize * rows;  // 网格总高度（像素）
}

// ============================================================================
// 辅助函数：绘制网格
// ============================================================================
// 参数：
//   grid        - 地图数据（二维 vector），0=可通行 1=障碍
//   states      - 每个格子的算法状态（UNVISITED/OPEN/CLOSED）
//   path        - 找到的路径坐标列表
//   currentNode - 当前正在扩展的节点坐标（用于橙色高亮）
//   found       - 是否已找到路径
//   cellSize    - 每个格子的像素边长
//   gridPixelW  - 网格总宽度（像素）
//   gridPixelH  - 网格总高度（像素）
// 目的：每帧把地图、Open/Closed/路径等画到屏幕上
void drawGrid(const std::vector<std::vector<int>>& grid,              // const& 表示只读引用，不拷贝
              const std::vector<std::vector<AStarStepper::CellState>>& states,
              const std::vector<std::pair<int, int>>& path,
              const std::pair<int, int>& currentNode,
              bool found,
              int cellSize, int gridPixelW, int gridPixelH) {

    // grid.size() 返回行数，static_cast<int> 把 size_t（无符号）转成 int（有符号）
    // 目的：获取地图行数，后续循环用
    int rows = static_cast<int>(grid.size());  // 行数

    // grid[0].size() 是第一行的列数（假设所有行列数相同）
    int cols = static_cast<int>(grid[0].size());  // 列数

    // ---- 第 1 步：画所有格子的底色 ----
    // 双重 for 循环：外层遍历行，内层遍历列
    for (int r = 0; r < rows; ++r) {           // ++r 是前置自增，比 r++ 在循环里稍快
        for (int c = 0; c < cols; ++c) {       // 遍历每一列

            // 计算格子的像素坐标：左边距 + 列号 * 格子大小
            int x = GRID_OFFSET_X + c * cellSize;  // 格子左上角 x 坐标
            int y = GRID_OFFSET_Y + r * cellSize;  // 格子左上角 y 坐标

            // 先设默认颜色为未访问
            Color cellColor = COLOR_UNVISITED;  // 默认浅灰白

            // 检查这个格子是不是障碍物
            if (grid[r][c] == 1) {  // 1 表示障碍
                cellColor = COLOR_OBSTACLE;  // 障碍物画黑色
            } else {
                // 不是障碍物，根据算法状态选颜色
                // switch 语句：多分支选择，根据 states[r][c] 的值跳转
                switch (states[r][c]) {
                    case AStarStepper::STATE_OPEN:          // 在 Open 表里
                        cellColor = COLOR_OPEN;             // 蓝色
                        break;  // break 跳出 switch，不继续执行后面的 case
                    case AStarStepper::STATE_CLOSED:        // 在 Closed 表里
                        cellColor = COLOR_CLOSED;           // 灰色
                        break;
                    case AStarStepper::STATE_UNVISITED:     // 还没访问
                    default:                                // default 是兜底分支
                        cellColor = COLOR_UNVISITED;        // 浅灰白
                        break;
                }
            }

            // raylib API：在 (x,y) 位置画一个 cellSize x cellSize 的填充矩形
            DrawRectangle(x, y, cellSize, cellSize, cellColor);
        }
    }

    // ---- 第 2 步：画网格线 ----
    // 目的：让格子边界清晰可见，用 DrawLine 画水平线和竖直线

    // 画水平线：行数+1 条（因为 N 行格子有 N+1 条边界线）
    for (int r = 0; r <= rows; ++r) {  // 注意是 <=，多画一条
        int y = GRID_OFFSET_Y + r * cellSize;  // 这条线的 y 坐标
        // DrawLine(起点x, 起点y, 终点x, 终点y, 颜色)
        DrawLine(GRID_OFFSET_X, y, GRID_OFFSET_X + gridPixelW, y, COLOR_GRID_LINE);
    }

    // 画竖直线：列数+1 条
    for (int c = 0; c <= cols; ++c) {
        int x = GRID_OFFSET_X + c * cellSize;  // 这条线的 x 坐标
        DrawLine(x, GRID_OFFSET_Y, x, GRID_OFFSET_Y + gridPixelH, COLOR_GRID_LINE);
    }

    // ---- 第 3 步：画当前正在扩展的节点（橙色描边） ----
    // 目的：高亮显示算法当前在处理哪个格子
    // 只在还没找到路径时画
    if (!found) {  // ! 是逻辑取反，found 为 false 时进入
        // currentNode 是 pair<int,int>，first=行 second=列
        // .second 访问 pair 的第二个元素（列号）
        int x = GRID_OFFSET_X + currentNode.second * cellSize;  // 当前格子的 x 像素坐标
        int y = GRID_OFFSET_Y + currentNode.first * cellSize;   // 当前格子的 y 像素坐标

        // raylib 没有设置线宽的 API，用画 3 层描边来模拟粗线
        // 每次外扩 1 像素，画 3 层叠加效果
        for (int i = 0; i < 3; ++i) {
            // DrawRectangleLines 画矩形边框（不填充内部）
            DrawRectangleLines(x - i, y - i, cellSize + 2 * i, cellSize + 2 * i, COLOR_CURRENT);
        }
    }

    // ---- 第 4 步：画最终路径（金色填充） ----
    // 目的：找到终点后把完整路径标出来
    // 条件：已找到路径 且 路径不为空
    if (found && !path.empty()) {  // && 是逻辑与，两个条件都为 true 才进入

        // range-based for 循环（C++11 特性）：遍历容器中每个元素
        // const auto& 让编译器自动推导类型，& 避免拷贝
        for (const auto& p : path) {  // p 是 pair<int,int>，表示路径上一个格子的坐标

            // 跳过起点和终点，它们后面用专门颜色画
            if (p.first == SOURCE_ROW && p.second == SOURCE_COL) continue;  // continue 跳过本次循环
            if (p.first == TARGET_ROW && p.second == TARGET_COL) continue;

            int x = GRID_OFFSET_X + p.second * cellSize;  // 路径格子的 x 像素坐标
            int y = GRID_OFFSET_Y + p.first * cellSize;   // 路径格子的 y 像素坐标

            // 画一个比格子略小的内框（缩进 2 像素），让底色透出来一点，更好看
            DrawRectangle(x + 2, y + 2, cellSize - 4, cellSize - 4, COLOR_PATH);
        }
    }

    // ---- 第 5 步：画起点（绿色方块 + 字母 "S"） ----
    // 用大括号 {} 包裹，让局部变量 x、y 的作用域只在这块内
    // 目的：避免和后面终点的变量名冲突
    {
        int x = GRID_OFFSET_X + SOURCE_COL * cellSize;  // 起点格子的 x 像素坐标
        int y = GRID_OFFSET_Y + SOURCE_ROW * cellSize;  // 起点格子的 y 像素坐标
        DrawRectangle(x, y, cellSize, cellSize, COLOR_START);  // 画绿色方块
        // DrawText(文字, x, y, 字号, 颜色)
        DrawText("S", x + cellSize / 4, y + cellSize / 4, cellSize / 2, WHITE);
    }

    // ---- 第 6 步：画终点（红色方块 + 字母 "G"） ----
    {
        int x = GRID_OFFSET_X + TARGET_COL * cellSize;  // 终点格子的 x 像素坐标
        int y = GRID_OFFSET_Y + TARGET_ROW * cellSize;  // 终点格子的 y 像素坐标
        DrawRectangle(x, y, cellSize, cellSize, COLOR_GOAL);  // 画红色方块
        DrawText("G", x + cellSize / 4, y + cellSize / 4, cellSize / 2, WHITE);
    }
}

// ============================================================================
// 辅助函数：绘制顶部信息栏
// ============================================================================
// 参数：
//   astar   - A* 步进器的只读引用（const& 不拷贝）
//   autoRun - 当前是否在自动运行
// 目的：在窗口顶部显示算法状态、步数、Open/Closed 计数
void drawInfoBar(const AStarStepper& astar, bool autoRun) {

    // 画背景条：从 (0,0) 开始，宽度=窗口宽，高度=70px
    DrawRectangle(0, 0, WINDOW_W, 70, { 240, 240, 245, 255 });  // 浅灰背景

    // 画标题文字
    // DrawText(文字内容, x, y, 字号, 颜色)
    DrawText("A* Path Planning Visualization", 20, 10, 24, { 40, 40, 60, 255 });  // 深蓝灰色标题

    // 根据算法当前状态决定显示什么文字
    std::string status;  // 声明一个空字符串

    if (astar.pathFound()) {                    // 如果找到路径了
        status = "PATH FOUND!";                 // 显示"找到了"
    } else if (astar.isDone()) {                // 否则如果算法结束了（没找到）
        status = "NO PATH (exhausted)";         // 显示"无路径"
    } else if (autoRun) {                       // 否则如果正在自动跑
        status = "Running...";                   // 显示"运行中"
    } else {                                    // 否则就是暂停状态
        status = "Paused (press SPACE to step, ENTER to auto)";
    }

    // 拼接完整信息字符串
    // std::to_string() 把 int 转成 string，+ 运算符拼接字符串
    std::string info = "Step: " + std::to_string(astar.getStepCount())   // 已执行步数
                     + "  |  Open: " + std::to_string(astar.getOpenCount())   // Open 表大小
                     + "  |  Closed: " + std::to_string(astar.getClosedCount()) // Closed 表大小
                     + "  |  " + status;  // 状态文字

    // .c_str() 把 std::string 转成 const char*（raylib 的 DrawText 需要 C 风格字符串）
    DrawText(info.c_str(), 20, 42, 18, { 80, 80, 100, 255 });  // 画状态信息
}

// ============================================================================
// 辅助函数：绘制底部图例
// ============================================================================
// 参数：cellSize 当前格子大小（当前未直接使用，但保留以备扩展）
// 目的：在窗口底部画一个颜色说明条，告诉用户每种颜色的含义
void drawLegend(int cellSize) {

    int y = WINDOW_H - 25;  // 图例的 y 坐标：距窗口底部 25 像素
    int x = 20;             // 图例的起始 x 坐标
    int box = 14;           // 颜色方块的大小（14x14 像素）

    // lambda 表达式（C++11）：定义一个匿名函数，捕获外部变量
    // [&] 表示按引用捕获所有外部变量（这里捕获 x，因为每次画完要更新 x）
    // 参数：const char* text（图例文字），Color color（方块颜色）
    auto drawItem = [&](const char* text, Color color) {
        DrawRectangle(x, y, box, box, color);  // 画颜色方块
        DrawText(text, x + box + 5, y - 1, 12, { 60, 60, 60, 255 });  // 画说明文字
        x += box + 5 + MeasureText(text, 12) + 15;  // x 右移，给下一个图例项腾出空间
        // MeasureText 测量文字在指定字号下的像素宽度
    };

    // 依次画每个图例项
    drawItem("Free", COLOR_UNVISITED);    // 浅灰白 = 未访问
    drawItem("Obstacle", COLOR_OBSTACLE); // 黑色 = 障碍
    drawItem("Open", COLOR_OPEN);         // 蓝色 = Open 表
    drawItem("Closed", COLOR_CLOSED);     // 灰色 = Closed 表
    drawItem("Current", COLOR_CURRENT);   // 橙色 = 当前节点
    drawItem("Path", COLOR_PATH);         // 金色 = 最终路径
    drawItem("Start", COLOR_START);       // 绿色 = 起点
    drawItem("Goal", COLOR_GOAL);         // 红色 = 终点
}

// ============================================================================
// 主函数
// ============================================================================
// 程序入口，操作系统调用这个函数启动程序
// 返回 int：0 表示正常退出，非 0 表示出错
int main() {

    // ---- 第 1 步：读取地图 ----

    // 创建 CSVReader 对象（栈上分配，函数结束自动销毁）
    CSVReader reader;

    // 调用 readRegion 从 CSV 读取指定区域
    // 参数：文件名, 起始行, 起始列, 行数, 列数
    // 跳过第 0 行（表头）和第 0 列（行号），读 30 行 30 列
    // auto 让编译器自动推导返回类型（这里是 vector<vector<int>>）
    auto grid = reader.readRegion(MAP_FILE, 1, 1, 30, 30);

    // 检查是否读取成功
    if (grid.empty()) {  // .empty() 检查 vector 是否为空
        // cerr 是标准错误输出流，用于输出错误信息
        return 1;  // 返回 1 表示程序异常退出
    }

    // 获取地图行列数
    int rows = static_cast<int>(grid.size());     // 行数 = vector 的元素个数
    int cols = static_cast<int>(grid[0].size());  // 列数 = 第一行的元素个数

    // ---- 第 2 步：计算格子尺寸 ----

    // 声明三个变量，初始值都设为 0
    int cellSize = 0, gridPixelW = 0, gridPixelH = 0;

    // 调用函数计算，通过引用参数带回结果
    calcCellSize(rows, cols, cellSize, gridPixelW, gridPixelH);

    // ---- 第 3 步：初始化 raylib 窗口 ----

    // InitWindow 创建一个 OpenGL 上下文 + GLFW 窗口
    // 参数：宽度, 高度, 标题
    InitWindow(WINDOW_W, WINDOW_H, "A* Path Planning - raylib");

    // SetTargetFPS 设置目标帧率，限制主循环每秒最多 60 次
    // 目的：避免吃满 CPU，同时让动画流畅
    SetTargetFPS(60);

    // ---- 第 4 步：创建 A* 步进器 ----

    // std::make_unique<C++14 引入）动态创建对象，返回 unique_ptr 智能指针
    // unique_ptr 会在指针不再被使用时自动 delete，不需要手动释放内存
    // 目的：方便 R 键重置时销毁旧对象、创建新对象
    auto pAstar = std::make_unique<AStarStepper>(grid, SOURCE_ROW, SOURCE_COL,
                                                  TARGET_ROW, TARGET_COL);

    // 是否自动连续步进的标志
    bool autoRun = false;  // 初始为暂停状态，等用户按键

    // ---- 第 5 步：主循环 ----

    // WindowShouldClose() 检测窗口是否应该关闭（用户按了 ESC 或点了 X）
    // 每次循环就是一帧，60 FPS 下每秒跑 60 次
    while (!WindowShouldClose()) {  // ! 是逻辑取反

        // ---- 输入处理 ----

        // IsKeyPressed 检测某键是否在当前帧被按下（按一下只触发一次）
        // KEY_SPACE 是 raylib 定义的空格键常量
        if (IsKeyPressed(KEY_SPACE)) {
            pAstar->step();  // -> 是通过指针调用成员函数（pAstar 是 unique_ptr）
        }

        // 回车键：切换自动运行模式（开/关）
        // KEY_ENTER 是主回车键，KEY_KP_ENTER 是小键盘的回车键
        if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {  // || 是逻辑或
            autoRun = !autoRun;  // 取反：true 变 false，false 变 true
        }

        // R 键：重置算法
        if (IsKeyPressed(KEY_R)) {
            // 用 make_unique 创建一个全新的 AStarStepper 对象
            // 旧的 unique_ptr 被覆盖时自动销毁旧对象（调用析构函数）
            pAstar = std::make_unique<AStarStepper>(grid, SOURCE_ROW, SOURCE_COL,
                                                    TARGET_ROW, TARGET_COL);
            autoRun = false;  // 重置后回到暂停状态
        }

        // 自动模式下每帧执行一步
        if (autoRun && !pAstar->isDone()) {  // 自动模式开着 且 算法还没结束
            pAstar->step();  // 推进一步
        }

        // ---- 绘制阶段 ----

        // BeginDrawing 通知 raylib：接下来的绘制命令都属于这一帧
        // 必须和 EndDrawing 配对使用
        BeginDrawing();

        // ClearBackground 用指定颜色清空整个画布
        // 目的：每帧重绘前清掉上一帧的内容，否则会有残影
        ClearBackground(COLOR_BG);

        // 画顶部信息栏（调用前面的辅助函数）
        // *pAstar 解引用 unique_ptr 得到对象本身
        drawInfoBar(*pAstar, autoRun);

        // 获取当前各格子的算法状态（每帧都获取最新值）
        auto states = pAstar->getCellStates();

        // 获取路径：如果找到了就取路径，否则返回空 vector
        // 三目运算符：pathFound() 为 true 调 getPath()，否则返回空 {}
        auto path = pAstar->pathFound() ? pAstar->getPath() : std::vector<std::pair<int, int>>{};

        // 调用绘制网格函数，把所有东西画出来
        drawGrid(grid, states, path, pAstar->getCurrentNode(),
                 pAstar->pathFound(), cellSize, gridPixelW, gridPixelH);

        // 画底部图例
        drawLegend(cellSize);

        // 画操作提示（右下角）
        DrawText("SPACE: step  |  ENTER: auto  |  R: reset  |  ESC: quit",
                 WINDOW_W - 420, WINDOW_H - 22, 12, { 120, 120, 130, 255 });

        // EndDrawing 通知 raylib：这一帧绘制结束
        // 它会交换前后缓冲区，把画好的内容显示到屏幕上
        EndDrawing();
    }

    // ---- 第 6 步：清理退出 ----

    // CloseWindow 关闭窗口并释放 OpenGL 上下文
    // 目的：程序退出前清理资源，避免句柄泄漏
    CloseWindow();

    return 0;  // 正常退出，返回 0
}
