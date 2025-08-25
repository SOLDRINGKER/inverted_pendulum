// src/pendulum_lqr_controller.cpp
#include "../include/HaitaiMotorController.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>
#include <signal.h>
#include <iomanip>
#include <thread>
#include <algorithm>

//=============================================================================
//                    ⚙️ 调试参数配置区域
//=============================================================================

// ===== 🎯 LQR控制增益矩阵 =====
constexpr float K1_THETA      = 34.78f;     // 倒立摆角度反馈增益
constexpr float K2_THETA_DOT  = 1.92f;      // 倒立摆角速度反馈增益
constexpr float K3_PHI        = 0.0032f;    // 飞轮角度反馈增益
constexpr float K4_PHI_DOT    = 0.101f;     // 飞轮角速度反馈增益

// ===== 🔌 硬件接口配置 =====
const std::string RS485_PORT = "/dev/ttyCH341USB0";
constexpr int SERIAL_BAUDRATE = 115200;
constexpr uint8_t PENDULUM_MOTOR_ID = 1;     // 摆杆关节电机ID
constexpr uint8_t FLYWHEEL_MOTOR_ID = 2;     // 飞轮电机ID

// ===== ⚡ 控制系统参数 =====
constexpr float CONTROL_FREQUENCY = 50.0f;   // 控制频率 50Hz

// ===== 🚀 启摆参数 =====
constexpr float PENDULUM_SWING_SPEED = 200.0f;       // 摆杆启摆速度 (RPM)
constexpr float FLYWHEEL_SWING_SPEED = 400.0f;       // 飞轮启摆速度 (RPM)
constexpr float SWING_COORDINATION_RATIO = 0.8f;     // 双电机协调比例

// ===== 🎛️  平衡控制参数 =====
constexpr float FLYWHEEL_MAX_BALANCE_SPEED = 1000.0f; // 飞轮最大平衡速度 (RPM)

// ===== 🎯 控制判定参数 =====
constexpr float BALANCE_ANGLE_THRESHOLD = 12.0f;     // 平衡角度阈值 (度)
constexpr float BALANCE_VELOCITY_THRESHOLD = 40.0f;  // 平衡速度阈值 (度/秒)

// ===== 📊 显示参数 =====
constexpr int DISPLAY_UPDATE_CYCLES = 20;            // 显示更新间隔
const std::string LOG_FILENAME = "pendulum_control_log.csv";

constexpr float DEG_TO_RAD = M_PI / 180.0f;

//=============================================================================
//                              🚀 程序主体部分
//=============================================================================

volatile bool g_running = true;

void signalHandler(int signal) {
    std::cout << "\n🛑 接收到停止信号 (" << signal << ")，正在安全关闭..." << std::endl;
    g_running = false;
}

class PendulumController {
public:
    enum class ControlMode {
        COORDINATED_SWING_UP,  // 协调启摆模式
        LQR_BALANCE_CONTROL,   // LQR平衡控制模式
        EMERGENCY_STOP         // 紧急停止
    };

    PendulumController() : 
        controller_(
            RS485_PORT,
            SERIAL_BAUDRATE,
            {{"pendulum", PENDULUM_MOTOR_ID}, {"flywheel", FLYWHEEL_MOTOR_ID}}
        ),
        current_mode_(ControlMode::COORDINATED_SWING_UP),
        swing_phase_counter_(0) {
        
        controller_.setErrorCallback([](const std::string& motorId, 
                                      HaitaiMotorController::ErrorCode /* error */, 
                                      const std::string& message) {
            std::cout << "❌ [错误] " << motorId << ": " << message << std::endl;
        });
    }

    ~PendulumController() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    bool initialize() {
        std::cout << "=== 🎯 动量轮倒立摆控制系统 ===" << std::endl;
        
        if (!controller_.open()) {
            std::cout << "❌ 串口连接失败: " << RS485_PORT << std::endl;
            return false;
        }
        std::cout << "✅ 串口连接成功" << std::endl;
        
        if (!detectMotors()) {
            return false;
        }
        
        if (!setCurrentPositionAsOrigin()) {
            return false;
        }
        
        if (!configureMotors()) {
            return false;
        }
        
        initializeLogging();
        
        std::cout << "\n✅ 系统初始化完成！" << std::endl;
        printSystemInfo();
        
        return true;
    }

    void run() {
        std::cout << "\n🚀 开始倒立摆控制..." << std::endl;
        
        auto control_period = std::chrono::milliseconds(static_cast<int>(1000.0f / CONTROL_FREQUENCY));
        auto next_time = std::chrono::steady_clock::now() + control_period;
        
        int cycle_count = 0;
        int balance_count = 0;
        
        current_mode_ = ControlMode::COORDINATED_SWING_UP;
        std::cout << "🔄 启动双电机协调启摆..." << std::endl;
        
        while (g_running) {
            SystemState state;
            if (!readSystemState(state)) {
                std::cout << "❌ 读取数据失败" << std::endl;
                break;
            }
            
            float upright_error = calculateUprightError(state.pendulum_angle);
            updateControlMode(state, upright_error);
            
            CoordinatedControl control = calculateCoordinatedControl(state, upright_error);
            
            if (!applyCoordinatedControl(control)) {
                std::cout << "❌ 协调控制失败" << std::endl;
                break;
            }
            
            bool is_balanced = (current_mode_ == ControlMode::LQR_BALANCE_CONTROL);
            if (is_balanced) balance_count++;
            
            logData(state, control, is_balanced, upright_error);
            
            if (++cycle_count % DISPLAY_UPDATE_CYCLES == 0) {
                displayStatus(state, control, is_balanced, upright_error);
            }
            
            std::this_thread::sleep_until(next_time);
            next_time += control_period;
        }
        
        safeShutdown();
        displayStatistics(cycle_count, balance_count);
    }

private:
    struct SystemState {
        float pendulum_angle;       // 摆杆单圈角度 (度)
        float pendulum_velocity;    // 摆杆角速度 (度/秒)
        float flywheel_angle;       // 飞轮单圈角度 (度)
        float flywheel_velocity;    // 飞轮角速度 (度/秒)
        float pendulum_voltage;
        float flywheel_voltage;
        float pendulum_current;
        float flywheel_current;
    };

    struct CoordinatedControl {
        float pendulum_speed;
        float flywheel_speed;
        std::string strategy;
    };

    HaitaiMotorController controller_;
    std::ofstream log_file_;
    ControlMode current_mode_;
    int swing_phase_counter_;

    bool detectMotors() {
        std::cout << "\n🔍 检测双电机..." << std::endl;
        
        HaitaiMotorController::DeviceInfo info;
        if (controller_.getDeviceInfo("pendulum", info) != HaitaiMotorController::ErrorCode::SUCCESS) {
            std::cout << "❌ 摆杆电机未找到" << std::endl;
            return false;
        }
        std::cout << "✅ 摆杆电机已连接" << std::endl;

        if (controller_.getDeviceInfo("flywheel", info) != HaitaiMotorController::ErrorCode::SUCCESS) {
            std::cout << "❌ 飞轮电机未找到" << std::endl;
            return false;
        }
        std::cout << "✅ 飞轮电机已连接" << std::endl;

        return true;
    }

    bool setCurrentPositionAsOrigin() {
        std::cout << "\n📍 设置当前位置为原点..." << std::endl;
        
        controller_.clearFaults("pendulum");
        controller_.clearFaults("flywheel");
        
        HaitaiMotorController::MotorPosition pendulum_pos, flywheel_pos;
        HaitaiMotorController::MotorStatus pendulum_status, flywheel_status;
        
        auto read_result1 = controller_.readSystemData("pendulum", pendulum_pos, pendulum_status);
        auto read_result2 = controller_.readSystemData("flywheel", flywheel_pos, flywheel_status);
        
        if (read_result1 == HaitaiMotorController::ErrorCode::SUCCESS) {
            std::cout << "   设置前摆杆角度: " << pendulum_pos.singleTurnAngle << "°" << std::endl;
        }
        
        if (read_result2 == HaitaiMotorController::ErrorCode::SUCCESS) {
            std::cout << "   设置前飞轮角度: " << flywheel_pos.singleTurnAngle << "°" << std::endl;
        }
        
        auto result1 = controller_.setCurrentPositionAsOrigin("pendulum");
        if (result1 != HaitaiMotorController::ErrorCode::SUCCESS) {
            std::cout << "   ❌ 摆杆原点设置失败" << std::endl;
            return false;
        }
        std::cout << "   ✅ 摆杆原点设置成功" << std::endl;
        
        auto result2 = controller_.setCurrentPositionAsOrigin("flywheel");
        if (result2 != HaitaiMotorController::ErrorCode::SUCCESS) {
            std::cout << "   ❌ 飞轮原点设置失败" << std::endl;
            return false;
        }
        std::cout << "   ✅ 飞轮原点设置成功" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        read_result1 = controller_.readSystemData("pendulum", pendulum_pos, pendulum_status);
        read_result2 = controller_.readSystemData("flywheel", flywheel_pos, flywheel_status);
        
        if (read_result1 == HaitaiMotorController::ErrorCode::SUCCESS) {
            std::cout << "   验证摆杆角度: " << pendulum_pos.singleTurnAngle << "°" << std::endl;
        }
        
        if (read_result2 == HaitaiMotorController::ErrorCode::SUCCESS) {
            std::cout << "   验证飞轮角度: " << flywheel_pos.singleTurnAngle << "°" << std::endl;
        }
        
        std::cout << "✅ 原点设置完成" << std::endl;
        return true;
    }

    bool configureMotors() {
        HaitaiMotorController::MotorLimits pendulum_limits = {-1800.0f, 1800.0f, 1000.0f};
        controller_.setMotorLimits("pendulum", pendulum_limits);
        
        HaitaiMotorController::MotorLimits flywheel_limits = {-100000.0f, 100000.0f, 3000.0f};
        controller_.setMotorLimits("flywheel", flywheel_limits);
        
        std::cout << "✅ 电机参数配置完成" << std::endl;
        return true;
    }

    void initializeLogging() {
        log_file_.open(LOG_FILENAME);
        if (log_file_.is_open()) {
            log_file_ << "时间(s),摆角(°),摆速(°/s),飞轮角(°),飞轮速(°/s),摆杆控制(RPM),飞轮控制(RPM),倒立误差(°),是否平衡,控制策略\n";
        }
    }

    void printSystemInfo() {
        std::cout << "\n📋 系统参数：" << std::endl;
        std::cout << "   🎯 LQR增益: K=[" << K1_THETA << ", " << K2_THETA_DOT 
                  << ", " << K3_PHI << ", " << K4_PHI_DOT << "]" << std::endl;
        std::cout << "   🚀 启摆速度: 摆杆" << PENDULUM_SWING_SPEED << "RPM + 飞轮" << FLYWHEEL_SWING_SPEED << "RPM" << std::endl;
        std::cout << "   ⚖️  平衡控制: 飞轮最大" << FLYWHEEL_MAX_BALANCE_SPEED << "RPM" << std::endl;
        std::cout << "   🎯 平衡阈值: " << BALANCE_ANGLE_THRESHOLD << "°" << std::endl;
        std::cout << "   📐 使用单圈角度控制" << std::endl;
    }

    bool readSystemState(SystemState& state) {
        HaitaiMotorController::MotorPosition pendulum_pos, flywheel_pos;
        HaitaiMotorController::MotorStatus pendulum_status, flywheel_status;
        
        auto result1 = controller_.readSystemData("pendulum", pendulum_pos, pendulum_status);
        auto result2 = controller_.readSystemData("flywheel", flywheel_pos, flywheel_status);
        
        if (result1 != HaitaiMotorController::ErrorCode::SUCCESS || 
            result2 != HaitaiMotorController::ErrorCode::SUCCESS) {
            return false;
        }
        
        state.pendulum_angle = pendulum_pos.singleTurnAngle;
        state.pendulum_velocity = pendulum_pos.speed * 6.0f;
        state.flywheel_angle = flywheel_pos.singleTurnAngle;
        state.flywheel_velocity = flywheel_pos.speed * 6.0f;
        state.pendulum_voltage = pendulum_status.voltage;
        state.flywheel_voltage = flywheel_status.voltage;
        state.pendulum_current = pendulum_status.current;
        state.flywheel_current = flywheel_status.current;
        
        return true;
    }

    float calculateUprightError(float angle) {
        while (angle > 180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        
        float error_to_pos180 = std::abs(angle - 180.0f);
        float error_to_neg180 = std::abs(angle - (-180.0f));
        
        return std::min(error_to_pos180, error_to_neg180);
    }

    void updateControlMode(const SystemState& state, float upright_error) {
        swing_phase_counter_++;
        
        switch (current_mode_) {
            case ControlMode::COORDINATED_SWING_UP:
                if (upright_error < BALANCE_ANGLE_THRESHOLD && 
                    std::abs(state.pendulum_velocity) < BALANCE_VELOCITY_THRESHOLD) {
                    current_mode_ = ControlMode::LQR_BALANCE_CONTROL;
                    std::cout << "\n🎯 切换到LQR平衡控制（误差:" << upright_error << "°）" << std::endl;
                }
                break;
                
            case ControlMode::LQR_BALANCE_CONTROL:
                if (upright_error > BALANCE_ANGLE_THRESHOLD * 2.5f) {
                    current_mode_ = ControlMode::COORDINATED_SWING_UP;
                    swing_phase_counter_ = 0;
                    std::cout << "\n🔄 重新协调启摆（误差:" << upright_error << "°）" << std::endl;
                }
                break;
                
            default:
                break;
        }
    }

    CoordinatedControl calculateCoordinatedControl(const SystemState& state, float upright_error) {
        CoordinatedControl control = {0.0f, 0.0f, "停止"};
        
        switch (current_mode_) {
            case ControlMode::COORDINATED_SWING_UP:
                control = calculateCoordinatedSwingUp(state, upright_error);
                break;
                
            case ControlMode::LQR_BALANCE_CONTROL:
                control = calculateLQRBalance(state);
                break;
                
            case ControlMode::EMERGENCY_STOP:
                control = {0.0f, 0.0f, "紧急停止"};
                break;
        }
        
        return control;
    }

    CoordinatedControl calculateCoordinatedSwingUp(const SystemState& state, float upright_error) {
        CoordinatedControl control;
        
        swing_phase_counter_++;
        
        if (upright_error > 90.0f) {
            if (state.pendulum_velocity >= 0) {
                control.pendulum_speed = PENDULUM_SWING_SPEED;
                control.flywheel_speed = -FLYWHEEL_SWING_SPEED;
                control.strategy = "大力上摆";
            } else {
                control.pendulum_speed = -PENDULUM_SWING_SPEED;
                control.flywheel_speed = FLYWHEEL_SWING_SPEED;
                control.strategy = "大力下摆";
            }
        }
        else if (upright_error > 30.0f) {
            if (state.pendulum_velocity >= 0) {
                control.pendulum_speed = PENDULUM_SWING_SPEED * 0.7f;
                control.flywheel_speed = -FLYWHEEL_SWING_SPEED * 0.8f;
                control.strategy = "适度上摆";
            } else {
                control.pendulum_speed = -PENDULUM_SWING_SPEED * 0.7f;
                control.flywheel_speed = FLYWHEEL_SWING_SPEED * 0.8f;
                control.strategy = "适度下摆";
            }
        }
        else {
            if (state.pendulum_velocity >= 0) {
                control.pendulum_speed = PENDULUM_SWING_SPEED * 0.2f;
                control.flywheel_speed = -FLYWHEEL_SWING_SPEED * 0.3f;
                control.strategy = "精细上摆";
            } else {
                control.pendulum_speed = -PENDULUM_SWING_SPEED * 0.2f;
                control.flywheel_speed = FLYWHEEL_SWING_SPEED * 0.3f;
                control.strategy = "精细下摆";
            }
        }
        
        control.pendulum_speed *= SWING_COORDINATION_RATIO;
        control.flywheel_speed *= SWING_COORDINATION_RATIO;
        
        if (swing_phase_counter_ > 150) {
            control.pendulum_speed *= -0.3f;
            control.flywheel_speed *= -0.3f;
            control.strategy += "(相位调整)";
        }
        
        if (swing_phase_counter_ > 200) {
            swing_phase_counter_ = 0;
        }
        
        return control;
    }

    CoordinatedControl calculateLQRBalance(const SystemState& state) {
        CoordinatedControl control;
        
        control.pendulum_speed = 0.0f;
        control.strategy = "LQR平衡";
        
        float signed_error = 0.0f;
        if (std::abs(state.pendulum_angle - 180.0f) < std::abs(state.pendulum_angle - (-180.0f))) {
            signed_error = state.pendulum_angle - 180.0f;
        } else {
            signed_error = state.pendulum_angle - (-180.0f);
        }
        
        float u = -(K1_THETA * (signed_error * DEG_TO_RAD) +
                   K2_THETA_DOT * (state.pendulum_velocity * DEG_TO_RAD) +
                   K3_PHI * (state.flywheel_angle * DEG_TO_RAD) +
                   K4_PHI_DOT * (state.flywheel_velocity * DEG_TO_RAD));
        
        control.flywheel_speed = u * 150.0f;
        control.flywheel_speed = std::max(-FLYWHEEL_MAX_BALANCE_SPEED, 
                                         std::min(FLYWHEEL_MAX_BALANCE_SPEED, control.flywheel_speed));
        
        return control;
    }

    bool applyCoordinatedControl(const CoordinatedControl& control) {
        HaitaiMotorController::MotorPosition pos1, pos2;
        
        auto result1 = controller_.speedClosedLoop("pendulum", control.pendulum_speed, pos1);
        auto result2 = controller_.speedClosedLoop("flywheel", control.flywheel_speed, pos2);
        
        return (result1 == HaitaiMotorController::ErrorCode::SUCCESS && 
                result2 == HaitaiMotorController::ErrorCode::SUCCESS);
    }

    void logData(const SystemState& state, const CoordinatedControl& control, 
                bool is_balanced, float upright_error) {
        static auto start_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - start_time).count();
        
        if (log_file_.is_open()) {
            log_file_ << std::fixed << std::setprecision(2)
                      << elapsed << ","
                      << state.pendulum_angle << ","
                      << state.pendulum_velocity << ","
                      << state.flywheel_angle << ","
                      << state.flywheel_velocity << ","
                      << control.pendulum_speed << ","
                      << control.flywheel_speed << ","
                      << upright_error << ","
                      << (is_balanced ? 1 : 0) << ","
                      << control.strategy << std::endl;
        }
    }

    void displayStatus(const SystemState& state, const CoordinatedControl& control,
                      bool is_balanced, float upright_error) {
        std::string mode_str;
        switch (current_mode_) {
            case ControlMode::COORDINATED_SWING_UP: mode_str = "协调启摆"; break;
            case ControlMode::LQR_BALANCE_CONTROL: mode_str = "LQR平衡"; break;
            case ControlMode::EMERGENCY_STOP: mode_str = "紧急停止"; break;
        }
        
        std::cout << std::fixed << std::setprecision(1)
                  << "\r🎯 摆角:" << std::setw(6) << state.pendulum_angle << "° "
                  << "误差:" << std::setw(5) << upright_error << "° "
                  << "摆速:" << std::setw(6) << state.pendulum_velocity << "°/s "
                  << "M1:" << std::setw(5) << control.pendulum_speed << "RPM "
                  << "M2:" << std::setw(5) << control.flywheel_speed << "RPM "
                  << "[" << mode_str << "] " << control.strategy << " "
                  << (is_balanced ? "🟢" : "🔴") << "    "
                  << std::flush;
    }

    void safeShutdown() {
        std::cout << "\n🛑 安全关闭..." << std::endl;
        
        for (int i = 0; i < 10; ++i) {
            CoordinatedControl stop_control = {0.0f, 0.0f, "停止中"};
            applyCoordinatedControl(stop_control);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        controller_.shutdownAllMotors();
        std::cout << "✅ 系统安全关闭" << std::endl;
    }

    void displayStatistics(int total_cycles, int balance_count) {
        float run_time = total_cycles / CONTROL_FREQUENCY;
        float balance_time = balance_count / CONTROL_FREQUENCY;
        float balance_rate = (run_time > 0) ? (balance_time / run_time) * 100.0f : 0.0f;
        
        std::cout << "\n\n=== 📊 控制统计 ===" << std::endl;
        std::cout << "⏱️  运行时间: " << run_time << "s" << std::endl;
        std::cout << "⚖️  倒立时间: " << balance_time << "s" << std::endl;
        std::cout << "📈 倒立成功率: " << balance_rate << "%" << std::endl;
        std::cout << "💾 数据: " << LOG_FILENAME << std::endl;
    }
};

//=============================================================================
//                              🎯 主函数
//=============================================================================

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        PendulumController controller;
        
        if (!controller.initialize()) {
            std::cout << "❌ 系统初始化失败！" << std::endl;
            return -1;
        }
        
        std::cout << "\n📋 系统准备完成：" << std::endl;
        std::cout << "   ✅ 原点已正确设置" << std::endl;
        std::cout << "   🚀 双电机协调启摆" << std::endl;
        std::cout << "   ⚖️  ±180°都能稳定" << std::endl;
        std::cout << "   📐 单圈角度控制" << std::endl;
        std::cout << "\n按 Enter 开始..." << std::endl;
        std::cin.get();
        
        controller.run();
        
    } catch (const std::exception& e) {
        std::cout << "❌ 程序异常: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "\n✅ 程序正常退出！" << std::endl;
    return 0;
}
