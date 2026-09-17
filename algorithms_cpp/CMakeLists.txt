cmake_minimum_required(VERSION 3.16)
project(astar_raylib)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)   # 关键：让 CMake 4.x 接受 raylib 的旧版本声明

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.0
    GIT_SUBMODULES ""
)
FetchContent_MakeAvailable(raylib)

add_executable(main main.cpp astar_stepper.cpp csv_reader.cpp)
target_link_libraries(main raylib)

# 编译后自动把 map.csv 复制到 build 目录
# 目的：运行 main.exe 时能从相对路径找到地图文件，不用手动复制
add_custom_command(TARGET main POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_SOURCE_DIR}/map.csv
            ${CMAKE_BINARY_DIR}/map.csv
)