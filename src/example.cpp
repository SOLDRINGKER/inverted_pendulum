#include "EmmV5Driver.h"
#include <iostream>
#include <unistd.h>

int main() {
    std::cout << "=== EMM_V5.0 Motor Control Example ===" << std::endl;
    
    // 创建驱动实例，1号电机
    EmmV5Driver motor("can0", 2);
    motor.setDebugMode(true); // 开启调试输出
    
    // 初始化CAN接口
    if (!motor.initialize()) {
        std::cerr << "Failed to initialize CAN interface!" << std::endl;
        return -1;
    }
    
    std::cout << "Motor driver initialized successfully!" << std::endl;
    
    // 读取电机状态
    MotorStatus_t status;
    if (motor.readMotorStatus(status)) {
        std::cout << "\n=== Motor Status ===" << std::endl;
        std::cout << "Enabled: " << (status.enabled ? "Yes" : "No") << std::endl;
        std::cout << "In Position: " << (status.in_position ? "Yes" : "No") << std::endl;
        std::cout << "Stalled: " << (status.stalled ? "Yes" : "No") << std::endl;
        std::cout << "Stall Protection: " << (status.stall_protection ? "Yes" : "No") << std::endl;
    }
    
    // 使能电机（不启用多机同步）
    std::cout << "\n=== Enabling Motor ===" << std::endl;
    if (motor.Emm_V5_En_Control(true, false)) {
        std::cout << "Motor enabled successfully!" << std::endl;
    } else {
        std::cerr << "Failed to enable motor!" << std::endl;
        return -1;
    }
    
    sleep(1);
    
    // 速度控制测试（包含多机同步标志）
    std::cout << "\n=== Speed Control Test ===" << std::endl;
    std::cout << "Starting CCW rotation at 1000 RPM..." << std::endl;
    if (motor.Emm_V5_Vel_Control(1, 1000, 10, false)) { // CCW, 1000RPM, 加速度10, 不启用多机同步
        std::cout << "Velocity control command sent!" << std::endl;
        sleep(3); // 运行3秒
        
        // 停止电机
        std::cout << "Stopping motor..." << std::endl;
        motor.Emm_V5_Stop_Now(false); // 不启用多机同步
        sleep(1);
    }
    
    // 位置控制测试（包含绝对/相对模式和多机同步标志）
    std::cout << "\n=== Position Control Test ===" << std::endl;
    
    // 重置位置为零点
    motor.Emm_V5_Reset_CurPos_To_Zero();
    sleep(1);
    
    // 相对位置控制：转5圈
    uint32_t pulses = EmmV5Driver::degreesToPulses(360.0f * 5, 16, 1.8f); // 5圈
    std::cout << "Position control: 5 turns CW (relative mode)" << std::endl;
    if (motor.Emm_V5_Pos_Control(0, 1000, 10, pulses, false, false)) { // CW, 1000RPM, 加速度10, 相对模式, 不启用多机同步
        std::cout << "Position control command sent!" << std::endl;
        
        // 监控运动进度
        for (int i = 0; i < 15; i++) {
            sleep(1);
            
            float position, velocity;
            if (motor.readCurrentPosition(position) && motor.readCurrentVelocity(velocity)) {
                std::cout << "Position: " << position << "°, Velocity: " << velocity << " RPM" << std::endl;
            }
            
            // 检查是否到位
            if (motor.readMotorStatus(status) && status.in_position) {
                std::cout << "Motor reached target position!" << std::endl;
                break;
            }
        }
    }
    
    sleep(2);
    
    // 绝对位置控制：回到0度
    std::cout << "\nReturning to 0° (absolute mode)" << std::endl;
    if (motor.Emm_V5_Pos_Control(0, 800, 5, 0, true, false)) { // CW方向, 绝对位置0度, 不启用多机同步
        std::cout << "Absolute position control command sent!" << std::endl;
        
        for (int i = 0; i < 15; i++) {
            sleep(1);
            
            float position;
            if (motor.readCurrentPosition(position)) {
                std::cout << "Position: " << position << "°" << std::endl;
            }
            
            if (motor.readMotorStatus(status) && status.in_position) {
                std::cout << "Motor returned to origin!" << std::endl;
                break;
            }
        }
    }
    
    // 失能电机
    std::cout << "\n=== Disabling Motor ===" << std::endl;
    if (motor.Emm_V5_En_Control(false, false)) {
        std::cout << "Motor disabled successfully!" << std::endl;
    }
    
    std::cout << "\n=== Test Completed ===" << std::endl;
    return 0;
}
