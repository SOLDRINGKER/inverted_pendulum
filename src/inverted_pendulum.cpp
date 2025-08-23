#include "EmmV5Driver.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>
#include <csignal>
#include <vector>
#include <memory>

// ================== 倒立摆参数区域（完全基于您的成功架构）==================

// LQR控制参数 - 基于您的MATLAB计算，但大幅缩放避免抖动
struct LQRParams {
    // 🎯 您的MATLAB计算结果
    std::array<float, 2> K_original = {{100.174230f, 10.045111f}};
    
    // 🔧 大幅缩放避免抖动 - 从您成功的Kp=3.5参考
    float scale_factor = 40.0f;  // 大幅缩放，类似您成功的example.cpp
    
    std::array<float, 2> K = {{
        K_original[0] / scale_factor,  // 100.174/50 = 2.003 (接近您成功的Kp=3.5的量级)
        K_original[1] / scale_factor   // 10.045/50 = 0.201 (接近您成功的Kd=0.03的量级)
    }};
};

// 起摆参数 - 基于您成功example.cpp的经验
struct SwingUpParams {
    float swing_amplitude = 200.0f;     // 🔧 大幅降低起摆幅度，避免过激
    float swing_frequency = 0.3f;       // 🔧 降低频率，更平稳
    float swing_duration = 12.0f;       // 延长起摆时间
    float balance_threshold = 10.0f;    // 平衡区域阈值（度）
};

// 🔥 控制参数 - 完全使用您example.cpp的成功参数
struct ControlParams {
    float max_speed = 1500.0f;       // 🔥 您要求的1500RPM
    float control_frequency = 100.0f; // 🔥 完全相同于您的example.cpp
    float duration = 20.0f;          // 🔥 完全相同于您的example.cpp
};

// ================== 全局变量 ==================

LQRParams lqr_params;
SwingUpParams swing_params;
ControlParams control_params;

volatile bool running = true;

void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n用户中断，正在停止..." << std::endl;
        running = false;
    }
}

// ================== 稳定的倒立摆控制器（基于您成功的架构）==================

class StablePendulumController {
public:
    StablePendulumController() {
        printControllerInfo();
    }
    
    // 🎯 LQR控制 - 使用类似您成功example.cpp的PD结构，但用LQR参数
    float lqrControl(float currentAngle, float targetAngle, float currentVelocity) {
        float angleError = targetAngle - currentAngle;
        
        // 🎯 正确的LQR控制律，但结构类似您成功的pidControl
        float velocityCommand = lqr_params.K[0] * angleError - lqr_params.K[1] * currentVelocity;
        
        // 🔥 完全按照您example.cpp的成功方式限幅
        velocityCommand = std::max(std::min(velocityCommand, control_params.max_speed), -control_params.max_speed);
        
        return velocityCommand;
    }
    
    // 起摆控制 - 基于您example.cpp的正弦轨迹思路
    float generateSwingTarget(float elapsedTime) {
        return swing_params.swing_amplitude * 
               std::sin(2.0f * M_PI * swing_params.swing_frequency * elapsedTime);
    }
    
    // 判断是否接近平衡位置
    bool isNearBalance(float currentAngle) {
        // 检查是否接近180度（倒立位置）
        float balance_error = std::abs(std::abs(currentAngle) - 180.0f);
        return balance_error < swing_params.balance_threshold;
    }
    
    void printControllerInfo() {
        std::cout << "\n=== 稳定倒立摆控制器（基于您成功的example.cpp）===" << std::endl;
        std::cout << "基于您example.cpp的成功架构 + 正确的LQR公式" << std::endl;
        std::cout << "原始K矩阵: [" << lqr_params.K_original[0] << ", " << lqr_params.K_original[1] << "]" << std::endl;
        std::cout << "缩放系数: /" << lqr_params.scale_factor << " (大幅缩放避免抖动)" << std::endl;
        std::cout << "实际K矩阵: [" << lqr_params.K[0] << ", " << lqr_params.K[1] << "]" << std::endl;
        std::cout << "起摆幅度: " << swing_params.swing_amplitude << "° (温和起摆)" << std::endl;
        std::cout << "最大速度: " << control_params.max_speed << "RPM" << std::endl;
    }
};

// ================== 主函数 - 🔥 完全照搬您成功的example.cpp ==================

int main() {
    std::signal(SIGINT, signalHandler);
    
    std::cout << "=== EMM_V5 稳定倒立摆控制（基于您成功的example.cpp架构）===" << std::endl;
    
    // 🔥 显示参数配置 - 完全按照您example.cpp的风格
    std::cout << "\n=== 控制参数 ===" << std::endl;
    std::cout << "LQR参数: K=[" << lqr_params.K[0] << ", " << lqr_params.K[1] << "] (大幅缩放)" << std::endl;
    std::cout << "起摆参数: 幅度=" << swing_params.swing_amplitude << "°, 频率=" 
              << swing_params.swing_frequency << "Hz (温和设置)" << std::endl;
    std::cout << "控制频率: " << control_params.control_frequency << "Hz" << std::endl;
    std::cout << "最大速度: " << control_params.max_speed << "RPM" << std::endl;
    std::cout << "运行时间: " << control_params.duration << "秒" << std::endl;
    
    // 🔥 创建驱动实例 - 完全按照您成功的example.cpp
    EmmV5Driver motor("can0", 1);
    motor.setDebugMode(true); // 🔥 和您example.cpp完全一样
    
    // 🔥 完全按照您成功的example.cpp初始化流程
    if (!motor.initialize()) {
        std::cerr << "Failed to initialize CAN interface!" << std::endl;
        return -1;
    }
    
    std::cout << "Motor driver initialized successfully!" << std::endl;
    
    // 🔥 完全按照您成功的example.cpp读取初始状态
    MotorStatus_t status;
    if (motor.readMotorStatus(status)) {
        std::cout << "\n=== 使能前电机状态 ===" << std::endl;
        std::cout << "使能状态: " << (status.enabled ? "已使能" : "未使能") << std::endl;
        std::cout << "到位状态: " << (status.in_position ? "到位" : "未到位") << std::endl;
        std::cout << "堵转状态: " << (status.stalled ? "堵转" : "正常") << std::endl;
    }
    
    sleep(1); // 🔥 和您example.cpp完全相同的延时
    
    // 🔥 完全按照您成功的example.cpp使能电机
    std::cout << "\n=== 使能电机 ===" << std::endl;
    if (motor.Emm_V5_En_Control(true, false)) {
        std::cout << "✅ 电机使能成功" << std::endl;
    } else {
        std::cerr << "❌ 电机使能失败" << std::endl;
        return -1;
    }
    
    sleep(1); // 🔥 和您example.cpp完全相同的延时
    
    // 🔥 完全按照您成功的example.cpp验证使能状态
    std::cout << "\n=== 使能后电机状态 ===" << std::endl;
    if (motor.readMotorStatus(status)) {
        std::cout << "✅ 状态读取成功" << std::endl;
        std::cout << "   使能状态: " << (status.enabled ? "已使能" : "未使能") << std::endl;
        std::cout << "   到位状态: " << (status.in_position ? "到位" : "未到位") << std::endl;
    }
    
    // 创建控制器
    StablePendulumController controller;
    
    // 记录零点位置 - 避免清零命令，防止CAN问题
    float zero_position = 0.0f;
    if (motor.readCurrentPosition(zero_position)) {
        std::cout << "零点位置: " << zero_position << "° (启动时摆杆位置)" << std::endl;
    }
    
    std::cout << "\n请确保摆杆垂直向下，按Enter开始稳定倒立摆控制..." << std::endl;
    std::cin.get();
    
    // 🔥 开始倒立摆控制 - 完全参考您example.cpp的成功循环结构
    std::cout << "\n=== 开始稳定倒立摆控制（按Ctrl+C停止）===" << std::endl;
    std::cout << "策略: 温和起摆 → 稳定LQR平衡控制" << std::endl;
    
    float loopPeriod = 1.0f / control_params.control_frequency; // 🔥 和您example.cpp相同的50Hz
    auto startTime = std::chrono::steady_clock::now();
    int displayCounter = 0;
    
    // 🔥 主控制循环 - 完全按照您example.cpp的成功结构
    while (running) {
        auto loopStartTime = std::chrono::steady_clock::now();
        float elapsedTime = std::chrono::duration<float>(loopStartTime - startTime).count();
        
        if (elapsedTime > control_params.duration) {
            break;
        }
        
        try {
            // 🔥 完全按照您example.cpp的成功方式读取当前状态
            float currentPosition, currentVelocity;
            
            if (motor.readCurrentPosition(currentPosition) && 
                motor.readCurrentVelocity(currentVelocity)) {
                
                // 计算相对于零点的角度
                float relativePosition = currentPosition - zero_position;
                
                float targetPosition;
                float velocityCommand;
                std::string control_mode;
                
                // 🔧 控制策略：温和起摆 + 稳定LQR平衡
                if (elapsedTime < swing_params.swing_duration && !controller.isNearBalance(relativePosition)) {
                    // 起摆阶段：使用类似您example.cpp的正弦轨迹思路
                    control_mode = "温和起摆";
                    targetPosition = controller.generateSwingTarget(elapsedTime);
                    
                    // 🔥 使用类似您example.cpp的PD控制进行起摆
                    float positionError = targetPosition - relativePosition;
                    velocityCommand = 1.0f * positionError - 0.01f * currentVelocity; // 温和的PD参数
                    
                } else {
                    // 平衡阶段：使用LQR控制
                    control_mode = "LQR平衡";
                    targetPosition = 180.0f;  // 目标：垂直向上
                    
                    // 🎯 使用LQR控制，但保持您example.cpp的成功结构
                    velocityCommand = controller.lqrControl(relativePosition, targetPosition, currentVelocity);
                }
                
                // 🔥 完全按照您example.cpp的成功方式发送控制命令
                uint8_t direction = (velocityCommand >= 0) ? 0 : 1;  // 0=CW, 1=CCW
                uint16_t speed = static_cast<uint16_t>(std::abs(velocityCommand));
                
                if (speed > 10) {  // 🔥 和您example.cpp完全一样的最小速度阈值
                    motor.Emm_V5_Vel_Control(direction, speed, 0, false);
                } else {
                    motor.Emm_V5_Stop_Now(false);
                }
                
                // 🔥 完全按照您example.cpp的成功方式显示状态
                if (displayCounter % (static_cast<int>(control_params.control_frequency / 5)) == 0) {
                    float error = targetPosition - relativePosition;
                    std::cout << "\r[" << control_mode << "] 时间: " << std::fixed << std::setprecision(1) << elapsedTime 
                              << "s, 目标=" << std::setw(6) << std::setprecision(1) << targetPosition 
                              << "°, 当前=" << std::setw(6) << relativePosition 
                              << "°, 误差=" << std::setw(5) << error << "°   " << std::flush;
                }
                
            } else {
                // 🔥 和您example.cpp完全一样的错误处理
                std::cerr << "\n❌ 无法读取电机状态" << std::endl;
                break;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "\n❌ 控制异常: " << e.what() << std::endl;
            break;
        }
        
        displayCounter++;
        
        // 🔥 完全按照您example.cpp的成功方式控制循环频率
        auto loopEndTime = std::chrono::steady_clock::now();
        auto loopDuration = std::chrono::duration<float>(loopEndTime - loopStartTime).count();
        float sleepTime = loopPeriod - loopDuration;
        
        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleepTime * 1000)));
        }
    }
    
    // 🔥 完全按照您example.cpp的成功方式停止电机
    std::cout << "\n\n=== Stopping Motor ===" << std::endl;
    motor.Emm_V5_Stop_Now(false);
    sleep(1);
    
    // 🔥 完全按照您example.cpp的成功方式失能电机  
    std::cout << "\n=== Disabling Motor ===" << std::endl;
    if (motor.Emm_V5_En_Control(false, false)) {
        std::cout << "✅ 电机失能成功" << std::endl;
    }
    
    std::cout << "\n=== 稳定倒立摆控制完成 ===" << std::endl;
    return 0;
}
