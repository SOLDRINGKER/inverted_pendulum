#include "EmmV5Driver.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>
#include <csignal>
#include <vector>
#include <memory>

// ================== 可修改参数区域 ==================

// PD控制参数
struct PIDParams {
    float kp = 3.5f;     // 位置比例增益，建议范围0.1-2.0
    float kd = 0.03f;    // 速度微分增益，建议范围0.01-0.2
};

// 正弦轨迹参数
struct SineParams {
    float amplitude = 100.0f;    // 振幅(度)，建议范围30-180
    float frequency = 0.5f;      // 频率(Hz)，建议范围0.1-2.0
    float phase = 0.0f;          // 相位(弧度)，0到2π
};

// 控制参数
struct ControlParams {
    float max_speed = 2500.0f;       // 最大速度(RPM)，建议不超过3000
    float control_frequency = 50.0f; // 控制频率(Hz)，建议20-100
    float duration = 20.0f;          // 运行时间(秒)
};

// ================== 程序配置 ==================

PIDParams pid_params;
SineParams sine_params;
ControlParams control_params;

// ================== 全局变量 ==================

volatile bool running = true;

void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n用户中断，正在停止..." << std::endl;
        running = false;
    }
}

// ================== 控制算法 ==================

float pidControl(float targetPosition, float currentPosition, float currentVelocity) {
    float positionError = targetPosition - currentPosition;
    float velocityCommand = pid_params.kp * positionError - pid_params.kd * currentVelocity;
    
    // 速度限幅
    velocityCommand = std::max(std::min(velocityCommand, control_params.max_speed), -control_params.max_speed);
    
    return velocityCommand;
}

float generateSineTarget(float elapsedTime) {
    return sine_params.amplitude * std::sin(2.0f * M_PI * sine_params.frequency * elapsedTime + sine_params.phase);
}

// ================== 数据记录和绘图 ==================

struct TrajectoryData {
    std::vector<float> timePoints;
    std::vector<float> targetPoints;
    std::vector<float> actualPoints;
};

void drawTrajectoryPlot(const TrajectoryData& data) {
    if (data.timePoints.empty()) return;
    
    const int width = 70;
    const int height = 20;
    
    float maxValue = -1000.0f;
    float minValue = 1000.0f;
    
    for (auto v : data.targetPoints) {
        maxValue = std::max(maxValue, v);
        minValue = std::min(minValue, v);
    }
    
    for (auto v : data.actualPoints) {
        maxValue = std::max(maxValue, v);
        minValue = std::min(minValue, v);
    }
    
    float range = maxValue - minValue;
    if (range < 1.0f) range = 1.0f;
    
    maxValue += range * 0.1f;
    minValue -= range * 0.1f;
    range = maxValue - minValue;
    
    std::cout << "\n========== 正弦轨迹跟随结果图 ==========" << std::endl;
    std::cout << "T=目标轨迹, A=实际位置, X=重合点" << std::endl;
    
    std::vector<std::string> plot(height, std::string(width, ' '));
    
    for (int i = 0; i < height; i++) {
        plot[i][0] = '|';
        plot[i][width-1] = '|';
    }
    
    for (int j = 0; j < width; j++) {
        plot[0][j] = '-';
        plot[height-1][j] = '-';
    }
    
    int midY = height / 2;
    for (int j = 0; j < width; j++) {
        plot[midY][j] = '-';
    }
    
    for (size_t i = 0; i < data.timePoints.size(); i += 2) {
        float t = data.timePoints[i];
        float targetVal = data.targetPoints[i];
        float actualVal = data.actualPoints[i];
        
        int x = static_cast<int>((t / data.timePoints.back()) * (width - 3)) + 1;
        int yTarget = static_cast<int>(((maxValue - targetVal) / range) * (height - 2)) + 1;
        int yActual = static_cast<int>(((maxValue - actualVal) / range) * (height - 2)) + 1;
        
        x = std::max(1, std::min(x, width - 2));
        yTarget = std::max(1, std::min(yTarget, height - 2));
        yActual = std::max(1, std::min(yActual, height - 2));
        
        plot[yTarget][x] = 'T';
        
        if (yTarget == yActual)
            plot[yActual][x] = 'X';
        else
            plot[yActual][x] = 'A';
    }
    
    for (const auto& line : plot) {
        std::cout << line << std::endl;
    }
    
    // 计算统计信息
    float sumError = 0.0f;
    float maxError = 0.0f;
    for (size_t i = 0; i < data.targetPoints.size(); i++) {
        float error = std::abs(data.targetPoints[i] - data.actualPoints[i]);
        sumError += error;
        maxError = std::max(maxError, error);
    }
    float avgError = sumError / data.targetPoints.size();
    
    std::cout << "\n性能统计: 平均误差=" << std::fixed << std::setprecision(2) 
              << avgError << "°, 最大误差=" << maxError << "°" << std::endl;
    std::cout << std::string(width, '=') << std::endl;
}

// ================== 主函数 ==================

int main() {
    std::signal(SIGINT, signalHandler);
    
    std::cout << "=== EMM_V5.0 正弦跟随PD控制 ===" << std::endl;
    
    // 显示参数配置
    std::cout << "\n=== 控制参数 ===" << std::endl;
    std::cout << "PD参数: Kp=" << pid_params.kp << ", Kd=" << pid_params.kd << std::endl;
    std::cout << "正弦轨迹: 振幅=" << sine_params.amplitude << "°, 频率=" 
              << sine_params.frequency << "Hz" << std::endl;
    std::cout << "控制频率: " << control_params.control_frequency << "Hz" << std::endl;
    std::cout << "运行时间: " << control_params.duration << "秒" << std::endl;
    
    // 创建驱动实例，1号电机 - 完全按照您成功的测试example
    EmmV5Driver motor("can0", 1);
    motor.setDebugMode(true); // 开启调试输出
    
    // 初始化CAN接口 - 保持与成功测试完全相同
    if (!motor.initialize()) {
        std::cerr << "Failed to initialize CAN interface!" << std::endl;
        return -1;
    }
    
    std::cout << "Motor driver initialized successfully!" << std::endl;
    
    // 读取电机初始状态 - 保持与成功测试完全相同
    MotorStatus_t status;
    if (motor.readMotorStatus(status)) {
        std::cout << "\n=== 使能前电机状态 ===" << std::endl;
        std::cout << "使能状态: " << (status.enabled ? "已使能" : "未使能") << std::endl;
        std::cout << "到位状态: " << (status.in_position ? "到位" : "未到位") << std::endl;
        std::cout << "堵转状态: " << (status.stalled ? "堵转" : "正常") << std::endl;
    }
    
    sleep(1);
    
    // 使能电机 - 保持与成功测试完全相同
    std::cout << "\n=== 使能电机 ===" << std::endl;
    if (motor.Emm_V5_En_Control(true, false)) {
        std::cout << "✅ 电机使能成功" << std::endl;
    } else {
        std::cerr << "❌ 电机使能失败" << std::endl;
        return -1;
    }
    
    sleep(1);
    
    // 验证使能后状态 - 确保电机正常工作
    std::cout << "\n=== 使能后电机状态 ===" << std::endl;
    if (motor.readMotorStatus(status)) {
        std::cout << "✅ 状态读取成功" << std::endl;
        std::cout << "   使能状态: " << (status.enabled ? "已使能" : "未使能") << std::endl;
        std::cout << "   到位状态: " << (status.in_position ? "到位" : "未到位") << std::endl;
    }
    
    // 数据记录
    TrajectoryData trajectoryData;
    
    // 开始正弦跟随控制
    std::cout << "\n=== 开始正弦轨迹跟随控制（按Ctrl+C停止）===" << std::endl;
    
    float loopPeriod = 1.0f / control_params.control_frequency;
    auto startTime = std::chrono::steady_clock::now();
    int displayCounter = 0;
    
    // 主控制循环
    while (running) {
        auto loopStartTime = std::chrono::steady_clock::now();
        float elapsedTime = std::chrono::duration<float>(loopStartTime - startTime).count();
        
        if (elapsedTime > control_params.duration) {
            break;
        }
        
        try {
            // 生成目标位置
            float targetPosition = generateSineTarget(elapsedTime);
            
            // 读取当前状态 - 基于成功的测试，这里应该能正常读取
            float currentPosition, currentVelocity;
            
            if (motor.readCurrentPosition(currentPosition) && 
                motor.readCurrentVelocity(currentVelocity)) {
                
                // PD控制算法
                float velocityCommand = pidControl(targetPosition, currentPosition, currentVelocity);
                
                // 发送速度控制命令
                uint8_t direction = (velocityCommand >= 0) ? 0 : 1;  // 0=CW, 1=CCW
                uint16_t speed = static_cast<uint16_t>(std::abs(velocityCommand));
                
                if (speed > 10) {  // 最小速度阈值
                    motor.Emm_V5_Vel_Control(direction, speed, 0, false);
                } else {
                    motor.Emm_V5_Stop_Now(false);
                }
                
                // 记录数据
                trajectoryData.timePoints.push_back(elapsedTime);
                trajectoryData.targetPoints.push_back(targetPosition);
                trajectoryData.actualPoints.push_back(currentPosition);
                
                // 显示状态（每秒5次）
                if (displayCounter % (static_cast<int>(control_params.control_frequency / 5)) == 0) {
                    float error = targetPosition - currentPosition;
                    std::cout << "\r时间: " << std::fixed << std::setprecision(1) << elapsedTime 
                              << "s, 目标=" << std::setw(6) << std::setprecision(1) << targetPosition 
                              << "°, 当前=" << std::setw(6) << currentPosition 
                              << "°, 误差=" << std::setw(5) << error << "°   " << std::flush;
                }
                
            } else {
                std::cerr << "\n❌ 无法读取电机状态" << std::endl;
                break;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "\n❌ 控制异常: " << e.what() << std::endl;
            break;
        }
        
        displayCounter++;
        
        // 精确控制循环频率
        auto loopEndTime = std::chrono::steady_clock::now();
        auto loopDuration = std::chrono::duration<float>(loopEndTime - loopStartTime).count();
        float sleepTime = loopPeriod - loopDuration;
        
        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleepTime * 1000)));
        }
    }
    
    // 停止电机 - 保持与成功测试完全相同的方式
    std::cout << "\n\n=== Stopping Motor ===" << std::endl;
    motor.Emm_V5_Stop_Now(false);
    sleep(1);
    
    // 失能电机 - 保持与成功测试完全相同的方式  
    std::cout << "\n=== Disabling Motor ===" << std::endl;
    if (motor.Emm_V5_En_Control(false, false)) {
        std::cout << "✅ 电机失能成功" << std::endl;
    }
    
    // 显示结果
    if (!trajectoryData.timePoints.empty()) {
        drawTrajectoryPlot(trajectoryData);
    }
    
    std::cout << "\n=== 正弦轨迹跟随控制完成 ===" << std::endl;
    return 0;
}
