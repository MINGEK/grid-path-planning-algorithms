cmake_minimum_required(VERSION 3.15)
project(AstarXlsx)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(OpenXLSX)

add_executable(A_star
    A_star.cpp
    csv_reader.cpp
)
target_link_libraries(A_star OpenXLSX::OpenXLSX)
