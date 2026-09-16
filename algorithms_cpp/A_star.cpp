/**
 * @file a_star.cpp
 * @brief A* global path planner implemented in C++17
 * @details Global grid-based path planning using the A* algorithm.
 * The grid map is loaded from a CSV file: 0 = free space, 1 = obstacle.
 * This module computes the shortest path from source to target and outputs the result to console.
 */


#include <iostream>
#include <vector>
#include <queue>      // 优先队列
#include <unordered_map> // 存 parent 和 cost
#include <algorithm>  // reverse
#include <array>
#include <cmath>     // sqrt
#include "csv_reader.h"


using namespace std;


// 格子坐标
struct Node {
    int row, col;
    double f, g, h;  // f = g + h

    bool operator>(const Node& other) const {
        if (f == other.f) {
            return g > other.g;  // f 相同，g 小的优先
        }
        return f > other.f;  // 优先队列用，f 小的先出
    }
};
// 八个方向的移动
struct Dir {
    int d_row, d_col;
    double cost;
};

// 八邻域
const double COST_STRAIGHT = 1.0;   // 直行 1
const double COST_DIAG = std::sqrt(2.0); // 斜行 sqrt(2)

const std::array<Dir,8> dirs = {{
    {-1,-1,COST_DIAG}, {-1,0,COST_STRAIGHT},{-1,1,COST_DIAG}, 
    {0,-1,COST_STRAIGHT}, {0,1,COST_STRAIGHT},
    {1,-1,COST_DIAG}, {1,0,COST_STRAIGHT}, {1,1,COST_DIAG}
}};


//  曼哈顿距离启发函数
double heuristic(int row1, int col1, int row2, int col2) {
    return abs(row1 - row2) + abs(col1 - col2);
}

// 把坐标转成唯一的 key（用于哈希表）
int key(int row, int col, int cols) {
    return row * cols + col;
}

// A* 算法
// 返回路径坐标列表，如果找不到返回空
vector<pair<int, int>> search(const vector<vector<int>>& grid,
                              int source_row, int source_col,
                              int target_row, int target_col) {
    int rows = grid.size();
    int cols = grid[0].size();

    // Open 表：优先队列，f 最小的先出
    // greater<Node> 让它变成"小顶堆"（默认是大顶堆）
    priority_queue<Node, vector<Node>, greater<Node>> open_list;

    // 存每个格子的 g 值（起点到这个格子的代价）
    unordered_map<int, double> gScore;

    // 存每个格子的父节点（用来回溯路径）
    unordered_map<int, int> parent;

    // 起点入队
    Node source;
    source.row = source_row;
    source.col = source_col;
    source.g = 0;
    source.h = heuristic(source_row, source_col, target_row, target_col);
    source.f = source.g + source.h;
    open_list.push(source);
    gScore[key(source_row, source_col, cols)] = 0;

    while (!open_list.empty()) {
        // 取出 f 最小的节点
        Node current = open_list.top();
        open_list.pop();

        // 到达终点，回溯路径
        if (current.row == target_row && current.col == target_col) {

            vector<pair<int, int>> path;

            int current_row = target_row, current_col = target_col;

            while (!(current_row == source_row && current_col == source_col)) {

                path.push_back({current_row, current_col});

                int k = key(current_row, current_col, cols);

                int parentK = parent[k];

                current_row = parentK / cols;

                current_col = parentK % cols;
            }
            path.push_back({source_row, source_col});

            reverse(path.begin(), path.end());

            return path;
        }

        if (current.g > gScore[key(current.row, current.col, cols)]) {
            // 如果当前节点的 g 值比记录的 g 值大，说明它已经被更优的路径访问过，跳过
            continue;
        }

        // 遍历八个方向
        for (int i = 0; i < 8; ++i) {

            int n_row = current.row + dirs[i].d_row;

            int n_col = current.col + dirs[i].d_col;

            // 越界检查
            if (n_row < 0 || n_row >= rows || n_col < 0 || n_col >= cols) continue;

            // 障碍物检查
            if (grid[n_row][n_col] == 1) continue;
            // 斜行检查：不能斜穿障碍
            if (dirs[i].cost == COST_DIAG) {
                int adj1_row = current.row;
                int adj1_col = n_col; // 水平方向的邻居
                int adj2_row = n_row; // 垂直方向的邻居
                int adj2_col = current.col;

                if (grid[adj1_row][adj1_col] == 1 || grid[adj2_row][adj2_col] == 1) {
                    continue; // 斜行被阻挡
                }
            }

            // 计算新的 g 值
            int tentativeG = current.g + dirs[i].cost;

            int nk = key(n_row, n_col, cols);

            // 如果这个格子还没探索过，或者新的 g 更小
            if (gScore.find(nk) == gScore.end() || tentativeG < gScore[nk]) {
                gScore[nk] = tentativeG;
                parent[nk] = key(current.row, current.col, cols);

                Node neighbor;
                neighbor.row = n_row;
                neighbor.col = n_col;
                neighbor.g = tentativeG;
                neighbor.h = heuristic(n_row, n_col, target_row, target_col);
                neighbor.f = neighbor.g + neighbor.h;
                open_list.push(neighbor);
            }
        }
    }

    // 没找到路径
    return {};
}



// 打印地图 + 路径
void printPath(const vector<vector<int>>& grid,
               const vector<pair<int, int>>& path,
               int startX, int startY,
               int goalX, int goalY) {
    int rows = grid.size();
    int cols = grid[0].size();

    // 先把路径标记出来
    vector<vector<char>> display(rows, vector<char>(cols, '.'));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (grid[i][j] == 1) display[i][j] = '#';
        }
    }

    // 标路径
    for (auto& p : path) {
        display[p.first][p.second] = '*';
    }

    // 标起点和终点
    display[startX][startY] = 'S';
    display[goalX][goalY] = 'G';

    // 打印
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            cout << display[i][j] << " ";
        }
        cout << endl;
    }
}

int main() {
    // 1. 读取地图
    CSVReader reader;
    auto grid = reader.readRegion("map.csv", 1, 1, 30, 30);

    if (grid.empty()) {
        cerr << "Failed to read map.csv" << endl;
        return 1;
    }

    int rows = grid.size();
    int cols = grid[0].size();
    cout << "Map size: " << rows << " x " << cols << endl;

    // 2. 设置起点和终点
    int source_row = 28, source_col =  4;    // 起点
    int target_row =  6, target_col = 24;    // 终点

    // 检查起点终点是不是障碍
    if (grid[source_row][source_col] == 1) {
        cerr << "Start is blocked!" << endl;
        return 1;
    }
    if (grid[target_row][target_col] == 1) {
        cerr << "Goal is blocked!" << endl;
        return 1;
    }

    // 3. 运行 A*
    cout << "\nRunning A* from (" << source_row << "," << source_col
         << ") to (" << target_row << "," << target_col << ")..." << endl;

    auto path = search(grid, source_row, source_col, target_row, target_col);

    // 4. 输出结果
    if (path.empty()) {
        cout << "\nNo path found!" << endl;
    } else {
        cout << "\nPath found! Length: " << path.size() << " steps" << endl;
        cout << "Path coordinates:" << endl;
        for (size_t i = 0; i < path.size(); ++i) {
            cout << "  (" << path[i].first << ", " << path[i].second << ")";
            if ((i + 1) % 5 == 0) cout << endl;
        }
        cout << "\n" << endl;
        printPath(grid, path, source_row, source_col, target_row, target_col);
    }

    return 0;
}
