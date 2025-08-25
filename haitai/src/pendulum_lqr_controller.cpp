// src/corrected_balance_pendulum.cpp
#include "../include/HaitaiMotorController.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>
#include <signal.h>
#include <iomanip>
#include <thread>

//=============================================================================
//                       🔧 可调节参数配置
//=============================================================================

// K矩阵参数 (基于您MATLAB计算的有效结果)
const float K_SCALE = 1.0f;                    // K矩阵整体放缩系数 (重要: 减小=响应更强)
const float K11 = 0.8090f / K_SCALE;           // 电机1→θ增益 (主要角度控制)
const float K12 = 0.4107f / K_SCALE;           // 电机1→θ̇增益 (角速度阻尼)
const float K13 = -0.0000f / K_SCALE;          // 电机1→φ增益
const float K14 = 0.0000f / K_SCALE;           // 电机1→φ̇增益
const float K21 = 0.0000f / K_SCALE;           // 电机2→θ增益
const float K22 = 0.0000f / K_SCALE;           // 电机2→θ̇增益
const float K23 = 0.0316f / K_SCALE;           // 电机2→φ增益 (飞轮位置)
const float K24 = 0.4169f / K_SCALE;           // 电机2→φ̇增益 (重要: 飞轮阻尼)

// 控制系统参数
const float CTRL_FREQ = 100.0f;                 // 控制频率Hz (影响响应速度)
const float VOLT_TO_RPM = 50.0f;               // 电压→RPM转换 (重要: 调节输出强度)
const float MAX_RPM = 400.0f;                  // 输出限制RPM (安全保护)

// 硬件参数
const std::string PORT = "/dev/ttyCH341USB0";  // 串口路径 (可修改)
const uint8_t MOTOR1_ID = 1;                   // 摆杆电机ID (可修改)
const uint8_t MOTOR2_ID = 2;                   // 飞轮电机ID (可修改)
const float SPEED_SCALE = 6.0f;                // 速度缩放系数

// 显示参数
const int DISPLAY_CYCLE = 25;                  // 显示间隔 (降低输出频率)
const std::string LOG_FILE = "fixed_balance.csv"; // 日志文件

const float DEG_RAD = M_PI / 180.0f;

volatile bool g_running = true;
void signalHandler(int sig) { (void)sig; g_running = false; }

class FixedLQRController {
private:
    struct State { float theta, theta_dot, phi, phi_dot; };
    struct Control { float m1_rpm, m2_rpm; };

    HaitaiMotorController controller_;
    std::ofstream log_;

public:
    FixedLQRController() : 
        controller_(PORT, 115200, {{"pendulum", MOTOR1_ID}, {"flywheel", MOTOR2_ID}}) {
        controller_.setErrorCallback([](const std::string& id, HaitaiMotorController::ErrorCode, const std::string& msg) {
            std::cout << "❌ " << id << ": " << msg << std::endl;
        });
    }

    ~FixedLQRController() { if (log_.is_open()) log_.close(); }

    bool init() {
        if (!controller_.open()) return false;
        
        HaitaiMotorController::DeviceInfo info;
        if (controller_.getDeviceInfo("pendulum", info) != HaitaiMotorController::ErrorCode::SUCCESS ||
            controller_.getDeviceInfo("flywheel", info) != HaitaiMotorController::ErrorCode::SUCCESS) {
            return false;
        }
        
        // 设置当前位置为0点
        controller_.clearFaults("pendulum");
        controller_.clearFaults("flywheel");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        if (controller_.setCurrentPositionAsOrigin("pendulum") != HaitaiMotorController::ErrorCode::SUCCESS ||
            controller_.setCurrentPositionAsOrigin("flywheel") != HaitaiMotorController::ErrorCode::SUCCESS) {
            return false;
        }
        
        // 配置单圈模式 (重要: ±180°范围)
        HaitaiMotorController::MotorLimits limits = {-180.0f, 180.0f, 600.0f};
        controller_.setMotorLimits("pendulum", limits);
        controller_.setMotorLimits("flywheel", limits);
        
        log_.open(LOG_FILE);
        if (log_.is_open()) {
            log_ << "时间(s),摆角(°),摆速(°/s),飞轮角(°),飞轮速(°/s),M1(RPM),M2(RPM),误差(°)\n";
        }
        
        std::cout << "✅ 初始化完成 (当前位置→0点)" << std::endl;
        return true;
    }

    void run() {
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0f / CTRL_FREQ));
        auto next_time = std::chrono::steady_clock::now() + period;
        
        int cycle = 0, good_cycles = 0;
        
        std::cout << "🎯 手动移动摆杆到±180°平衡位置..." << std::endl;
        
        while (g_running) {
            State state;
            if (!readState(state)) break;
            
            float error = calcError(state.theta);
            Control ctrl = calcFixedLQR(state);        // 修正的双向LQR控制
            
            if (!applyControl(ctrl)) break;
            
            if (error < 15.0f) good_cycles++;
            logData(state, ctrl, error);
            
            if (++cycle % DISPLAY_CYCLE == 0) {
                displayStatus(state, ctrl, error);
            }
            
            std::this_thread::sleep_until(next_time);
            next_time += period;
        }
        
        shutdown();
        showStats(cycle, good_cycles);
    }

private:
    bool readState(State& s) {
        HaitaiMotorController::MotorPosition pos1, pos2;
        HaitaiMotorController::MotorStatus status1, status2;
        
        if (controller_.readSystemData("pendulum", pos1, status1) != HaitaiMotorController::ErrorCode::SUCCESS ||
            controller_.readSystemData("flywheel", pos2, status2) != HaitaiMotorController::ErrorCode::SUCCESS) {
            return false;
        }
        
        // 读取单圈角度
        s.theta = pos1.singleTurnAngle;
        s.theta_dot = pos1.speed * SPEED_SCALE;
        s.phi = pos2.singleTurnAngle;
        s.phi_dot = pos2.speed * SPEED_SCALE;
        
        return true;
    }

    float calcError(float angle) {
        while (angle > 180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        return std::min(std::abs(angle - 180.0f), std::abs(angle - (-180.0f)));
    }

    // 🔧 修正的双向LQR控制 (解决单方向问题)
    Control calcFixedLQR(const State& s) {
        // ✅ 修正的角度误差计算 (避免偏向性选择)
        float theta_err;
        float dist_to_pos180 = std::abs(s.theta - 180.0f);
        float dist_to_neg180 = std::abs(s.theta - (-180.0f));
        
        if (dist_to_pos180 < dist_to_neg180) {
            theta_err = s.theta - 180.0f;       // 目标+180°
        } else if (dist_to_neg180 < dist_to_pos180) {
            theta_err = s.theta - (-180.0f);    // 目标-180°
        } else {
            // ✅ 关键修正：距离相等时，基于当前角度选择更合理的目标
            if (s.theta >= 0) {
                theta_err = s.theta - 180.0f;   // 正角度→目标+180°
            } else {
                theta_err = s.theta - (-180.0f); // 负角度→目标-180°
            }
        }
        
        // 状态向量 (基于文档的状态空间方法)
        float x1 = theta_err * DEG_RAD;       
        float x2 = s.theta_dot * DEG_RAD;     
        float x3 = s.phi * DEG_RAD;           
        float x4 = s.phi_dot * DEG_RAD;       
        
        // LQR控制律: u = -Kx (参考文档连续控制策略)
        float u1 = -(K11*x1 + K12*x2 + K13*x3 + K14*x4);
        float u2 = -(K21*x1 + K22*x2 + K23*x3 + K24*x4);
        
        Control ctrl;
        ctrl.m1_rpm = u1 * VOLT_TO_RPM;      
        ctrl.m2_rpm = u2 * VOLT_TO_RPM;
        
        // 饱和限制
        ctrl.m1_rpm = std::max(-MAX_RPM, std::min(MAX_RPM, ctrl.m1_rpm));
        ctrl.m2_rpm = std::max(-MAX_RPM, std::min(MAX_RPM, ctrl.m2_rpm));
        
        return ctrl;
    }

    bool applyControl(const Control& ctrl) {
        HaitaiMotorController::MotorPosition pos1, pos2;
        return (controller_.speedClosedLoop("pendulum", ctrl.m1_rpm, pos1) == HaitaiMotorController::ErrorCode::SUCCESS &&
                controller_.speedClosedLoop("flywheel", ctrl.m2_rpm, pos2) == HaitaiMotorController::ErrorCode::SUCCESS);
    }

    void logData(const State& s, const Control& ctrl, float err) {
        static auto start = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        
        if (log_.is_open()) {
            log_ << std::fixed << std::setprecision(2)
                 << elapsed << "," << s.theta << "," << s.theta_dot << ","
                 << s.phi << "," << s.phi_dot << ","
                 << ctrl.m1_rpm << "," << ctrl.m2_rpm << "," << err << std::endl;
        }
    }

    void displayStatus(const State& s, const Control& ctrl, float err) {
        std::cout << std::fixed << std::setprecision(1)
                  << "\r🎯θ:" << std::setw(6) << s.theta << "° "
                  << "误差:" << std::setw(4) << err << "° "
                  << "θ̇:" << std::setw(5) << s.theta_dot << "°/s "
                  << "φ̇:" << std::setw(5) << s.phi_dot << "°/s "
                  << "M1:" << std::setw(5) << ctrl.m1_rpm << " "
                  << "M2:" << std::setw(5) << ctrl.m2_rpm << " "
                  << (err < 8.0f ? "🟢" : err < 15.0f ? "🟡" : "🔴")
                  << std::flush;
    }

    void shutdown() {
        for (int i = 0; i < 10; ++i) {
            Control stop = {0.0f, 0.0f};
            applyControl(stop);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        controller_.shutdownAllMotors();
        if (log_.is_open()) log_.close();
    }

    void showStats(int total, int good) {
        float time = total / CTRL_FREQ;
        float rate = time > 0 ? (good / CTRL_FREQ / time) * 100.0f : 0.0f;
        
        std::cout << "\n\n📊 运行" << time << "s, 平衡率" << rate << "%" << std::endl;
        std::cout << "🔧 调试: K_SCALE=" << K_SCALE << " VOLT_TO_RPM=" << VOLT_TO_RPM << std::endl;
    }
};

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        FixedLQRController controller;
        
        if (!controller.init()) {
            std::cout << "❌ 初始化失败" << std::endl;
            return -1;
        }
        
        std::cout << "📋 修正双向控制的LQR平衡系统" << std::endl;
        std::cout << "🔧 K_SCALE=" << K_SCALE << " VOLT_TO_RPM=" << VOLT_TO_RPM << std::endl;
        std::cout << "按Enter开始..." << std::endl;
        std::cin.get();
        
        controller.run();
        
    } catch (const std::exception& e) {
        std::cout << "❌ " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
