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
    message(STATUS "EmmV5Driver source files not found - library will not be built")
    set(EMM_V5_DRIVER_AVAILABLE FALSE)
endif()

# 检查并创建可执行文件（仅当相应源文件存在时）
set(EXECUTABLES_BUILT "")

# 检查example.cpp
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/example.cpp")
    if(EMM_V5_DRIVER_AVAILABLE)
        add_executable(motor_example src/example.cpp)
        target_link_libraries(motor_example PRIVATE emm_v5_driver)
        list(APPEND EXECUTABLES_BUILT "motor_example")
        message(STATUS "Added target: motor_example")
    else()
        message(WARNING "example.cpp found but EmmV5Driver library not available")
    endif()
endif()

# 检查motor_can_test.cpp
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/motor_can_test.cpp")
    if(EMM_V5_DRIVER_AVAILABLE)
        add_executable(motor_can_test src/motor_can_test.cpp)
        target_link_libraries(motor_can_test PRIVATE emm_v5_driver)
        list(APPEND EXECUTABLES_BUILT "motor_can_test")
        message(STATUS "Added target: motor_can_test")
    endif()
endif()

# 检查can_debug_tool.cpp
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/can_debug_tool.cpp")
    if(EMM_V5_DRIVER_AVAILABLE)
        add_executable(can_debug_tool src/can_debug_tool.cpp)
        target_link_libraries(can_debug_tool PRIVATE emm_v5_driver)
        list(APPEND EXECUTABLES_BUILT "can_debug_tool")
        message(STATUS "Added target: can_debug_tool")
    endif()
endif()

# 检查simple_can_test.cpp
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/simple_can_test.cpp")
    if(EMM_V5_DRIVER_AVAILABLE)
        add_executable(simple_can_test src/simple_can_test.cpp)
        target_link_libraries(simple_can_test PRIVATE emm_v5_driver)
        list(APPEND EXECUTABLES_BUILT "simple_can_test")
        message(STATUS "Added target: simple_can_test")
    endif()
endif()

# 如果没有找到任何可执行程序源文件，创建一个简单的测试程序
if(NOT EXECUTABLES_BUILT)
    message(STATUS "No executable source files found, creating simple test program")
    
    # 创建一个简单的测试文件
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/simple_test.cpp"
"#include <iostream>

int main() {
    std::cout << \"Hello from inverted_pendulum project!\" << std::endl;
    std::cout << \"To use EMM_V5 motor driver, please add the following files:\" << std::endl;
    std::cout << \"  - include/EmmV5Driver.h\" << std::endl;
    std::cout << \"  - src/EmmV5Driver.cpp\" << std::endl;
    std::cout << \"Then you can create motor control programs.\" << std::endl;
    return 0;
}
")
    
    add_executable(simple_test "${CMAKE_CURRENT_BINARY_DIR}/simple_test.cpp")
    list(APPEND EXECUTABLES_BUILT "simple_test")
    message(STATUS "Added target: simple_test (fallback)")
endif()

# 添加自定义目标用于设置CAN接口
add_custom_target(setup_can
    COMMAND sudo modprobe gs_usb
    COMMAND sudo ip link set can0 type can bitrate 1000000
    COMMAND sudo ip link set up can0
    COMMENT "Setting up CAN interface with 1M bitrate"
    VERBATIM
)

# 添加自定义目标用于检查CAN接口状态
add_custom_target(check_can
    COMMAND ip -details link show can0
    COMMENT "Checking CAN interface status"
    VERBATIM
)

# 添加清理CAN接口的目标
add_custom_target(cleanup_can
    COMMAND sudo ip link set down can0
    COMMENT "Shutting down CAN interface"
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

# 显示将要构建的目标
if(EXECUTABLES_BUILT)
    message(STATUS "Executables to build: ${EXECUTABLES_BUILT}")
else()
    message(STATUS "No executables will be built")
endif()

if(EMM_V5_DRIVER_AVAILABLE)
    message(STATUS "EmmV5Driver library: Available")
else()
    message(STATUS "EmmV5Driver library: Not available")
endif()

message(STATUS "Custom targets: setup_can, check_can, cleanup_can")
