cmake_minimum_required(VERSION 3.5)
project(cspc_lidar)

# 设置 C++ 版本为 C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加源代码文件
file(GLOB SOURCES ./*.cpp)

# 添加可执行文件生成规则
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接pthread库
find_package(Threads REQUIRED)
target_link_libraries(cspc_lidar PRIVATE Threads::Threads)
