cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(Emm_V5_Control_Suite)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -pthread")

# 设置Debug和Release模式的编译选项
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# 添加头文件搜索路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 查找需要的系统库
find_package(Threads REQUIRED)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

# 创建输出目录
file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# 收集共享的源文件
set(COMMON_SOURCES
    src/EmmV5Driver.cpp
)

# 收集头文件
set(HEADERS
    include/EmmV5Driver.h
)

# ================================
# 目标1: 倒立摆控制程序
# ================================
add_executable(inverted_pendulum 
    src/inverted_pendulum.cpp
    ${COMMON_SOURCES}
    ${HEADERS}
)

target_link_libraries(inverted_pendulum 
    Threads::Threads    # 线程库
    rt                  # 实时库
)

# ================================
# 目标2: PD控制正弦跟随程序
# ================================
add_executable(example
    src/example.cpp
    ${COMMON_SOURCES}
    ${HEADERS}
)

target_link_libraries(example 
    Threads::Threads    # 线程库
    rt                  # 实时库
)

# ================================
# 设置目标属性
# ================================
set_target_properties(inverted_pendulum PROPERTIES
    OUTPUT_NAME "inverted_pendulum"
)

set_target_properties(example PROPERTIES
    OUTPUT_NAME "example"
)

# ================================
# 添加自定义目标
# ================================

# 清理目标
add_custom_target(clean-all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

# 安装目标（可选）
install(TARGETS inverted_pendulum example
    RUNTIME DESTINATION bin
)

# ================================
# 打印编译信息
# ================================
message(STATUS "=== Emm_V5 Control Suite ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Output Directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "Targets:")
message(STATUS "  - inverted_pendulum (倒立摆控制)")
message(STATUS "  - example (PD控制正弦跟随)")
message(STATUS "Common Sources: ${COMMON_SOURCES}")
