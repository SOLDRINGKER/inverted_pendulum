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

# 倒立摆控制程序 - 新增
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/inverted_pendulum.cpp")
    if(EMM_V5_DRIVER_AVAILABLE)
        add_executable(inverted_pendulum src/inverted_pendulum.cpp)
        target_link_libraries(inverted_pendulum PRIVATE emm_v5_driver)
        list(APPEND EXECUTABLES_BUILT "inverted_pendulum")
        message(STATUS "Added target: inverted_pendulum (倒立摆控制程序)")
    else()
        message(WARNING "inverted_pendulum.cpp found but EmmV5Driver library not available")
    endif()
endif()

# 原有的example程序 - 保持兼容
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/example.cpp")
    if(EMM_V5_DRIVER_AVAILABLE)
        add_executable(emm_motor_example src/example.cpp)
        target_link_libraries(emm_motor_example PRIVATE emm_v5_driver)
        list(APPEND EXECUTABLES_BUILT "emm_motor_example")
        message(STATUS "Added target: emm_motor_example (电机控制演示程序)")
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
    std::cout << \"EMM_V5 倒立摆控制系统\" << std::endl;
    std::cout << \"可用程序:\" << std::endl;
    std::cout << \"  - inverted_pendulum: 倒立摆控制程序\" << std::endl;
    std::cout << \"  - emm_motor_example: EMM_V5电机控制演示\" << std::endl;
    std::cout << \"请确保EmmV5Driver.cpp和EmmV5Driver.h文件存在。\" << std::endl;
    std::cout << \"\\n项目结构应该是:\" << std::endl;
    std::cout << \"  src/inverted_pendulum.cpp - 倒立摆主程序\" << std::endl;
    std::cout << \"  src/example.cpp - 原有的电机测试程序\" << std::endl;
    std::cout << \"  src/EmmV5Driver.cpp - 电机驱动实现\" << std::endl;
    std::cout << \"  include/EmmV5Driver.h - 电机驱动头文件\" << std::endl;
    return 0;
}
")
    
    add_executable(basic_test "${CMAKE_CURRENT_BINARY_DIR}/basic_test.cpp")
    list(APPEND EXECUTABLES_BUILT "basic_test")
    message(STATUS "Added target: basic_test (fallback)")
endif()

# 添加自定义目标用于CAN接口管理
add_custom_target(can_setup
    COMMAND sudo modprobe gs_usb
    COMMAND sudo ip link set can0 type can bitrate 1000000
    COMMAND sudo ip link set up can0
    COMMENT "Setting up CAN interface with 1M bitrate for EMM_V5 motor control"
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

# 显示使用帮助的目标
add_custom_target(show_usage
    COMMAND echo "EMM_V5 倒立摆控制系统使用说明:"
    COMMAND echo "================================"
    COMMAND echo "1. 设置CAN接口: make can_setup"
    COMMAND echo "2. 检查CAN状态: make can_check"
    COMMAND echo "3. 运行倒立摆程序: ./bin/inverted_pendulum"
    COMMAND echo "4. 运行电机测试: ./bin/emm_motor_example"
    COMMAND echo "5. 清理CAN接口: make can_cleanup"
    COMMAND echo ""
    COMMAND echo "文件结构要求:"
    COMMAND echo "  src/inverted_pendulum.cpp - 倒立摆主程序"
    COMMAND echo "  src/example.cpp          - 电机测试程序"
    COMMAND echo "  src/EmmV5Driver.cpp      - 电机驱动实现文件"
    COMMAND echo "  include/EmmV5Driver.h    - 电机驱动头文件"
    COMMAND echo ""
    COMMAND echo "倒立摆控制说明:"
    COMMAND echo "  - 程序启动时电机当前角度将作为0度基准"
    COMMAND echo "  - 目标是将摆杆从当前位置摆到直立(180度)并保持平衡"
    COMMAND echo "  - 使用双环PID控制：外环位置控制 + 内环角度PD控制"
    COMMAND echo "  - 按Ctrl+C可以安全停止程序"
    COMMENT "Displaying usage information for inverted pendulum control system"
    VERBATIM
)

# 添加倒立摆专用的快速启动目标
if("inverted_pendulum" IN_LIST EXECUTABLES_BUILT)
    add_custom_target(run_pendulum
        COMMAND echo "启动倒立摆控制系统..."
        COMMAND echo "请确保:"
        COMMAND echo "1. CAN接口已设置 (make can_setup)"
        COMMAND echo "2. EMM_V5电机已连接并供电"
        COMMAND echo "3. 摆杆处于期望的初始位置"
        COMMAND echo ""
        COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/inverted_pendulum
        DEPENDS inverted_pendulum
        COMMENT "Running inverted pendulum control system"
        VERBATIM
    )
endif()

# 打印构建信息
message(STATUS "=== 倒立摆项目构建配置 ===")
message(STATUS "项目名称: ${PROJECT_NAME}")
message(STATUS "项目版本: ${PROJECT_VERSION}")
message(STATUS "构建类型: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++标准: ${CMAKE_CXX_STANDARD}")
message(STATUS "源码目录: ${CMAKE_SOURCE_DIR}")
message(STATUS "构建目录: ${CMAKE_BINARY_DIR}")

if(EXECUTABLES_BUILT)
    message(STATUS "将构建的可执行文件: ${EXECUTABLES_BUILT}")
    if("inverted_pendulum" IN_LIST EXECUTABLES_BUILT)
        message(STATUS "  倒立摆程序: ./bin/inverted_pendulum")
    endif()
    if("emm_motor_example" IN_LIST EXECUTABLES_BUILT)
        message(STATUS "  电机测试程序: ./bin/emm_motor_example")
    endif()
else()
    message(STATUS "没有找到可构建的程序")
endif()

if(EMM_V5_DRIVER_AVAILABLE)
    message(STATUS "EmmV5Driver库: 可用")
else()
    message(STATUS "EmmV5Driver库: 不可用 - 请添加EmmV5Driver.cpp和EmmV5Driver.h")
endif()

message(STATUS "自定义目标: can_setup, can_check, can_cleanup, show_usage")
if("inverted_pendulum" IN_LIST EXECUTABLES_BUILT)
    message(STATUS "快速启动: make run_pendulum")
endif()
message(STATUS "=== 构建配置完成 ===")
