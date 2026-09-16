#include "csv_reader.h"
#include <fstream>
#include <sstream>

std::vector<std::vector<int>> CSVReader::readInt(const std::string& filename) {
    std::vector<std::vector<int>> data;

    std::ifstream file(filename);
    if (!file.is_open()) {
        return data;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        data.push_back(splitLineToInt(line));
    }

    file.close();
    return data;
}

std::vector<std::vector<int>> CSVReader::readRegion(const std::string& filename,
                                                     int startRow, int startCol,
                                                     int rows, int cols) {
    // 先全部读进来
    auto allData = readInt(filename);
    if (allData.empty()) return {};

    std::vector<std::vector<int>> result;

    // 从 startRow 开始，取 rows 行
    for (int i = startRow; i < startRow + rows && i < (int)allData.size(); ++i) {
        std::vector<int> row;
        // 从 startCol 开始，取 cols 列
        for (int j = startCol; j < startCol + cols && j < (int)allData[i].size(); ++j) {
            row.push_back(allData[i][j]);
        }
        result.push_back(row);
    }

    return result;
}

int CSVReader::readCell(const std::string& filename, int row, int col) {
    auto data = readRegion(filename, row, col, 1, 1);
    if (data.empty() || data[0].empty()) return -1;
    return data[0][0];
}

std::vector<int> CSVReader::splitLineToInt(const std::string& line, char delimiter) {
    std::vector<int> fields;
    std::stringstream ss(line);
    std::string field;

    while (std::getline(ss, field, delimiter)) {
        if (field.empty()) {
            fields.push_back(0);
        } else {
            try {
                fields.push_back(std::stoi(field));
            } catch (...) {
                fields.push_back(0);
            }
        }
    }

    return fields;
}
