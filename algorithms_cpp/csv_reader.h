#ifndef CSV_READER_H
#define CSV_READER_H

#include <string>
#include <vector>

class CSVReader {
public:
    // 读取整个 csv，返回 int 二维数组（自动跳过表头和行号列）
    std::vector<std::vector<int>> readInt(const std::string& filename);

    // 读取指定区域：从 startRow 行 startCol 列开始，读 rows 行 cols 列
    // 行号、列号都从 0 开始算
    std::vector<std::vector<int>> readRegion(const std::string& filename,
                                              int startRow, int startCol,
                                              int rows, int cols);

    // 读取单个单元格的值
    int readCell(const std::string& filename, int row, int col);

private:
    std::vector<int> splitLineToInt(const std::string& line, char delimiter = ',');
};

#endif
