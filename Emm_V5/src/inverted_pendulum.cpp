#include "EmmV5Driver.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>
#include <csignal>
#include <vector>
#include <memory>

// ================== 倒立摆参数配置区域 ==================

// LQR控制参数
struct LQRParams {
    std::array<float, 2> K_original = {{100.174230f, 10.045111f}};  // MATLAB计算的原始增益
    float scale_factor = 10.0f;  // 增益缩放系数，越大越温和
    
    std::array<float, 2> K = {{
        K_original[0] / scale_factor,  // 位置增益
        K_original[1] / scale_factor   // 速度增益
    }};
    
    // 柔性区间参数
    float deadzone_deg = 3.0f;      // 死区范围(度) - 几乎不控制
    float soft_zone_deg = 10.0f;    // 柔性区范围(度) - 渐进控制
    float max_zone_deg = 20.0f;     // 最大区范围(度) - 正常控制
};

// 起摆参数
struct SwingUpParams {
    float swing_amplitude = 200.0f;     // 起摆振幅(度)
    float swing_frequency = 0.3f;       // 起摆频率(Hz)
    float swing_duration = 12.0f;       // 起摆持续时间(秒)
    float balance_threshold = 15.0f;    // 平衡判定阈值(度)
};

// 控制参数
struct ControlParams {
    float max_speed = 1000.0f;      // 最大控制速度(RPM)
    float control_frequency = 100.0f; // 控制频率(Hz)
    float duration = 30.0f;         // 运行时间(秒)
};

// ================== 全局变量 ==================

LQRParams lqr_params;
SwingUpParams swing_params;
ControlParams control_params;

volatile bool running = true;

void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n停止控制..." << std::endl;
        running = false;
    }
}

// ================== 柔性倒立摆控制器 ==================

class FlexiblePendulumController {
public:
    FlexiblePendulumController() {}
    
    // 柔性LQR控制
    float flexibleLQRControl(float currentAngle, float targetAngle, float currentVelocity) {
        float angleError = targetAngle - currentAngle;
        
        // 角度跳变处理
        while (angleError > 180.0f) angleError -= 360.0f;
        while (angleError < -180.0f) angleError += 360.0f;
        
        float abs_error = std::abs(angleError);
        
        // 柔性控制系数计算
        float flexibility_factor = 1.0f;
        
        if (abs_error < lqr_params.deadzone_deg) {
            flexibility_factor = 0.1f;  // 死区：10%控制力
        } else if (abs_error < lqr_params.soft_zone_deg) {
            // 柔性区：10%→80%渐进
            float progress = (abs_error - lqr_params.deadzone_deg) / 
                           (lqr_params.soft_zone_deg - lqr_params.deadzone_deg);
            flexibility_factor = 0.1f + progress * 0.7f;
        } else if (abs_error < lqr_params.max_zone_deg) {
            flexibility_factor = 0.8f;  // 正常区：80%控制力
        } else {
            flexibility_factor = 1.0f;  // 强控制区：100%控制力
        }
        
        // LQR控制律
        float velocityCommand = (lqr_params.K[0] * angleError - lqr_params.K[1] * currentVelocity) * flexibility_factor;
        
        // 速度限幅
        velocityCommand = std::max(std::min(velocityCommand, control_params.max_speed), -control_params.max_speed);
        
        return velocityCommand;
    }
    
    // 起摆轨迹生成
    float generateSwingTarget(float elapsedTime) {
        return swing_params.swing_amplitude * 
               std::sin(2.0f * M_PI * swing_params.swing_frequency * elapsedTime);
    }
    
    // 平衡位置判定 - 支持±180度
    bool isNearBalance(float currentAngle) {
        float distance_to_upright = std::min(
            std::abs(std::abs(currentAngle) - 180.0f),
            std::abs(std::abs(currentAngle) - 180.0f)
        );
        return distance_to_upright < swing_params.balance_threshold;
    }
    
    // 获取当前控制状态
    std::string getFlexibilityStatus(float currentAngle, float targetAngle) {
        float angleError = targetAngle - currentAngle;
        while (angleError > 180.0f) angleError -= 360.0f;
        while (angleError < -180.0f) angleError += 360.0f;
        
        float abs_error = std::abs(angleError);
        
        if (abs_error < lqr_params.deadzone_deg) {
            return "死区10%";
        } else if (abs_error < lqr_params.soft_zone_deg) {
            return "柔性区";
        } else if (abs_error < lqr_params.max_zone_deg) {
            return "正常80%";
        } else {
            return "强控制";
        }
    }
};

// ================== 主函数 ==================

int main() {
    std::signal(SIGINT, signalHandler);
    
    std::cout << "=== EMM_V5 柔性倒立摆控制 ===" << std::endl;
    
    // 创建驱动实例
    EmmV5Driver motor("can0", 1);
    motor.setDebugMode(false);
    
    if (!motor.initialize()) {
        std::cerr << "CAN初始化失败!" << std::endl;
        return -1;
    }
    
    // 使能电机
    if (!motor.Emm_V5_En_Control(true, false)) {
        std::cerr << "电机使能失败!" << std::endl;
        return -1;
    }
    
    sleep(1);
    
    // 记录零点位置
    float zero_position = 0.0f;
    if (motor.readCurrentPosition(zero_position)) {
        std::cout << "零点: " << zero_position << "°" << std::endl;
    }
    
    FlexiblePendulumController controller;
    
    std::cout << "参数: 振幅=" << swing_params.swing_amplitude << "°, 缩放=" << lqr_params.scale_factor 
              << ", 死区=" << lqr_params.deadzone_deg << "°" << std::endl;
    std::cout << "摆杆垂直向下，按Enter开始..." << std::endl;
    std::cin.get();
    
    std::cout << "开始控制 (Ctrl+C停止)" << std::endl;
    
    float loopPeriod = 1.0f / control_params.control_frequency;
    auto startTime = std::chrono::steady_clock::now();
    int displayCounter = 0;
    
    while (running) {
        auto loopStartTime = std::chrono::steady_clock::now();
        float elapsedTime = std::chrono::duration<float>(loopStartTime - startTime).count();
        
        if (elapsedTime > control_params.duration) break;
        
        try {
            float currentPosition, currentVelocity;
            
            if (motor.readCurrentPosition(currentPosition) && 
                motor.readCurrentVelocity(currentVelocity)) {
                
                float relativePosition = currentPosition - zero_position;
                float targetPosition;
                float velocityCommand;
                std::string control_mode;
                
                // 控制逻辑：起摆 → 柔性平衡
                if (elapsedTime < swing_params.swing_duration && !controller.isNearBalance(relativePosition)) {
                    // 起摆阶段
                    control_mode = "起摆";
                    targetPosition = controller.generateSwingTarget(elapsedTime);
                    
                    float positionError = targetPosition - relativePosition;
                    velocityCommand = 1.0f * positionError - 0.01f * currentVelocity;
                    
                } else {
                    // 柔性平衡阶段
                    targetPosition = 180.0f;
                    control_mode = controller.getFlexibilityStatus(relativePosition, targetPosition);
                    
                    velocityCommand = controller.flexibleLQRControl(relativePosition, targetPosition, currentVelocity);
                }
                
                // 发送控制命令
                uint8_t direction = (velocityCommand >= 0) ? 0 : 1;
                uint16_t speed = static_cast<uint16_t>(std::abs(velocityCommand));
                
                if (speed > 10) {
                    motor.Emm_V5_Vel_Control(direction, speed, 0, false);
                } else {
                    motor.Emm_V5_Stop_Now(false);
                }
                
                // 状态显示
                if (displayCounter % 20 == 0) {
                    float upright_error = std::min(
                        std::abs(std::abs(relativePosition) - 180.0f),
                        180.0f - std::abs(std::abs(relativePosition) - 180.0f)
                    );
                    
                    std::cout << "\r[" << control_mode << "] " 
                              << std::fixed << std::setprecision(1) << elapsedTime << "s "
                              << "角度=" << relativePosition << "° "
                              << "误差=" << upright_error << "° "
                              << "控制=" << (int)velocityCommand << "   " << std::flush;
                }
                
            } else {
                std::cerr << "\n读取状态失败" << std::endl;
                break;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "\n控制异常: " << e.what() << std::endl;
            break;
        }
        
        displayCounter++;
        
        auto loopEndTime = std::chrono::steady_clock::now();
        auto loopDuration = std::chrono::duration<float>(loopEndTime - loopStartTime).count();
        float sleepTime = loopPeriod - loopDuration;
        
        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleepTime * 1000)));
        }
    }
    
    // 停止电机
    motor.Emm_V5_Stop_Now(false);
    sleep(1);
    motor.Emm_V5_En_Control(false, false);
    
    std::cout << "\n控制完成" << std::endl;
    return 0;
}
