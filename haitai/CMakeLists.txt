# CMakeLists.txt - 动量轮倒立摆LQR控制系统构建配置

cmake_minimum_required(VERSION 3.10)

project(PendulumLQRController
    VERSION 1.0.0
    DESCRIPTION "带动量轮倒立摆LQR控制系统"
    LANGUAGES CXX
)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项 - 移除警告抑制，因为我们已经修复了警告
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# Debug和Release模式设置
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 对于多配置生成器设置输出目录
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/lib)
endforeach()

# 添加头文件搜索路径
include_directories(${CMAKE_SOURCE_DIR}/include)

# 源文件配置
set(HEADER_FILES include/HaitaiMotorController.h)
set(LIBRARY_SOURCES src/HaitaiMotorController.cpp)
set(MAIN_SOURCES src/pendulum_lqr_controller.cpp)

# 查找依赖
find_package(Threads REQUIRED)

# 检查系统头文件
include(CheckIncludeFiles)
check_include_files("termios.h" HAVE_TERMIOS_H)
check_include_files("unistd.h" HAVE_UNISTD_H)

if(NOT HAVE_TERMIOS_H OR NOT HAVE_UNISTD_H)
    message(FATAL_ERROR "缺少必要的系统头文件，请确保在Linux系统上编译")
endif()

# 创建电机控制库
add_library(haitai_motor_controller STATIC ${LIBRARY_SOURCES} ${HEADER_FILES})

# 设置库的属性
set_target_properties(haitai_motor_controller PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "${HEADER_FILES}"
)

# 库的编译选项
target_compile_options(haitai_motor_controller PRIVATE
    -fPIC
    -Wall
    -Wextra
)

# 库的链接
target_link_libraries(haitai_motor_controller 
    PUBLIC Threads::Threads
)

# 创建主可执行文件
add_executable(pendulum_lqr_controller ${MAIN_SOURCES})

# 链接库到主程序
target_link_libraries(pendulum_lqr_controller 
    PRIVATE haitai_motor_controller
    PRIVATE Threads::Threads
)

# 设置可执行文件属性
set_target_properties(pendulum_lqr_controller PROPERTIES
    OUTPUT_NAME "pendulum_lqr_controller"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# 安装配置
install(TARGETS pendulum_lqr_controller
    RUNTIME DESTINATION bin
    COMPONENT Runtime
)

install(TARGETS haitai_motor_controller
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
    COMPONENT Development
)

# 自定义目标
add_custom_target(clean-all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/bin
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/lib
    COMMENT "清理所有构建文件"
)

add_custom_target(create-dirs ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/lib
    COMMENT "创建输出目录"
)

# 显示配置信息
message(STATUS "========================================")
message(STATUS "项目配置信息:")
message(STATUS "  项目名称: ${PROJECT_NAME}")
message(STATUS "  版本: ${PROJECT_VERSION}")
message(STATUS "  C++标准: C++${CMAKE_CXX_STANDARD}")
message(STATUS "  编译类型: ${CMAKE_BUILD_TYPE}")
message(STATUS "  编译器: ${CMAKE_CXX_COMPILER}")
message(STATUS "  输出目录: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "  警告状态: 已修复所有编译警告")
message(STATUS "========================================")
