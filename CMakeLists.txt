cmake_minimum_required(VERSION 3.16)

# 项目信息
project(inverted_pendulum VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# 编译器设置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-g -O0 -DDEBUG)
    else()
        add_compile_options(-O3 -DNDEBUG)
    endif()
endif()

# 包含目录
include_directories(include)

# 查找依赖
find_package(Threads REQUIRED)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 检查EmmV5Driver源文件是否存在
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/EmmV5Driver.cpp" AND 
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/EmmV5Driver.h")
   
    message(STATUS "Found EmmV5Driver source files")
    
    # 创建EmmV5Driver静态库
    add_library(emm_v5_driver STATIC
        src/EmmV5Driver.cpp
        include/EmmV5Driver.h
    )
    
    # 设置库的属性
    target_include_directories(emm_v5_driver PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    
    target_link_libraries(emm_v5_driver
        PUBLIC
            Threads::Threads
    )
    
    target_compile_features(emm_v5_driver PUBLIC cxx_std_17)
    
    # 库可用标志
    set(EMM_V5_DRIVER_AVAILABLE TRUE)
    message(STATUS "EmmV5Driver library will be built")
    
else()
    message(WARNING "EmmV5Driver source files not found - library will not be built")
    set(EMM_V5_DRIVER_AVAILABLE FALSE)
endif()

# 创建可执行文件
set(EXECUTABLES_BUILT "")

# 主要的example程序
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/example.cpp")
    if(EMM_V5_DRIVER_AVAILABLE)
        add_executable(emm_motor_example src/example.cpp)
        target_link_libraries(emm_motor_example PRIVATE emm_v5_driver)
        list(APPEND EXECUTABLES_BUILT "emm_motor_example")
        message(STATUS "Added target: emm_motor_example")
    else()
        message(WARNING "example.cpp found but EmmV5Driver library not available")
    endif()
endif()

# 如果没有找到任何可执行程序源文件，创建一个基础测试程序
if(NOT EXECUTABLES_BUILT)
    message(STATUS "Creating basic test program")
    
    # 创建一个基础测试文件
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/basic_test.cpp"
"#include <iostream>

int main() {
    std::cout << \"EMM_V5 Motor Control Library Test\" << std::endl;
    std::cout << \"Available programs:\" << std::endl;
    std::cout << \"  - emm_motor_example: EMM_V5电机控制演示\" << std::endl;
    std::cout << \"Please ensure EmmV5Driver.cpp and EmmV5Driver.h are present.\" << std::endl;
    return 0;
}
")
    
    add_executable(basic_test "${CMAKE_CURRENT_BINARY_DIR}/basic_test.cpp")
    list(APPEND EXECUTABLES_BUILT "basic_test")
    message(STATUS "Added target: basic_test (fallback)")
endif()

# 添加自定义目标用于CAN接口管理 - 避免使用保留名称
add_custom_target(can_setup
    COMMAND sudo modprobe gs_usb
    COMMAND sudo ip link set can0 type can bitrate 1000000
    COMMAND sudo ip link set up can0
    COMMENT "Setting up CAN interface with 1M bitrate for Geschwister Schneider adapter"
    VERBATIM
)

add_custom_target(can_check
    COMMAND ip -details link show can0
    COMMENT "Checking CAN interface status"
    VERBATIM
)

add_custom_target(can_cleanup
    COMMAND sudo ip link set down can0
    COMMENT "Shutting down CAN interface"
    VERBATIM
)

# 显示配置帮助的目标 - 重命名避免保留名称冲突
add_custom_target(show_usage
    COMMAND echo "EMM_V5 Motor Control Usage:"
    COMMAND echo "1. 设置CAN接口: make can_setup"
    COMMAND echo "2. 检查CAN状态: make can_check"
    COMMAND echo "3. 运行示例程序: ./bin/emm_motor_example"
    COMMAND echo "4. 清理CAN接口: make can_cleanup"
    COMMAND echo ""
    COMMAND echo "文件结构要求:"
    COMMAND echo "  include/EmmV5Driver.h  - 电机驱动头文件"
    COMMAND echo "  src/EmmV5Driver.cpp    - 电机驱动实现文件"
    COMMAND echo "  src/example.cpp        - 示例程序"
    COMMENT "Displaying usage information"
    VERBATIM
)

# 打印构建信息
message(STATUS "=== Build Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Source directory: ${CMAKE_SOURCE_DIR}")
message(STATUS "Binary directory: ${CMAKE_BINARY_DIR}")

if(EXECUTABLES_BUILT)
    message(STATUS "Executables to build: ${EXECUTABLES_BUILT}")
else()
    message(STATUS "No executables will be built")
endif()

if(EMM_V5_DRIVER_AVAILABLE)
    message(STATUS "EmmV5Driver library: Available")
else()
    message(STATUS "EmmV5Driver library: Not available - please add EmmV5Driver.cpp and EmmV5Driver.h")
endif()

message(STATUS "Custom targets: can_setup, can_check, can_cleanup, show_usage")
message(STATUS "=== End Configuration ===")
