#ifndef EMM_V5_DRIVER_H
#define EMM_V5_DRIVER_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <cstdint>
#include <chrono>
#include <thread>

#define ABS(x) ((x) > 0 ? (x) : -(x))

// 系统参数枚举 - 与原始Emm_V5.h完全一致
typedef enum {
    S_VER   = 0,    /* 读取固件版本和对应的硬件版本 */
    S_RL    = 1,    /* 读取读取相电阻和相电感 */
    S_PID   = 2,    /* 读取PID参数 */
    S_VBUS  = 3,    /* 读取总线电压 */
    S_CPHA  = 5,    /* 读取相电流 */
    S_ENCL  = 7,    /* 读取经过线性化校准后的编码器值 */
    S_TPOS  = 8,    /* 读取电机目标位置角度 */
    S_VEL   = 9,    /* 读取电机实时转速 */
    S_CPOS  = 10,   /* 读取电机实时位置角度 */
    S_PERR  = 11,   /* 读取电机位置误差角度 */
    S_FLAG  = 13,   /* 读取使能/到位/堵转状态标志位 */
    S_Conf  = 14,   /* 读取驱动参数 */
    S_State = 15,   /* 读取系统状态参数 */
    S_ORG   = 16,   /* 读取正在回零/回零失败状态标志位 */
} SysParams_t;

// 电机状态结构体
typedef struct {
    bool enabled;         // 电机使能状态
    bool in_position;     // 电机到位状态 
    bool stalled;         // 电机堵转状态
    bool stall_protection; // 堵转保护状态
} MotorStatus_t;

// 回零状态结构体
typedef struct {
    bool encoder_ready;     // 编码器就绪状态
    bool calibration_ready; // 校准表就绪状态
    bool homing;           // 正在回零标志
    bool homing_failed;    // 回零失败标志
} HomingStatus_t;

class EmmV5Driver {
private:
    int can_socket_;           // CAN套接字
    uint8_t motor_address_;    // 电机地址
    std::string can_interface_; // CAN接口名称
    bool debug_mode_;          // 调试模式
    
    // CAN通信相关方法 - 基于data.c的can_SendCmd实现
    void can_SendCmd(const std::vector<uint8_t>& cmd);
    bool receiveCanFrame(std::vector<uint8_t>& data, uint32_t timeout_ms = 1000);
    void debugPrint(const std::string& message);

public:
    EmmV5Driver(const std::string& can_interface = "can0", uint8_t motor_address = 1);
    ~EmmV5Driver();
    
    // 初始化和清理
    bool initialize();
    void close();
    
    // === 基于原始Emm_V5.c的函数接口 ===
    
    // 将当前位置清零
    void Emm_V5_Reset_CurPos_To_Zero();
    
    // 解除堵转保护
    void Emm_V5_Reset_Clog_Pro();
    
    // 读取参数
    void Emm_V5_Read_Sys_Params(SysParams_t s);
    
    // 发送命令修改开环/闭环控制模式
    void Emm_V5_Modify_Ctrl_Mode(bool svF, uint8_t ctrl_mode);
    
    // 电机使能控制
    bool Emm_V5_En_Control(bool state, bool snF);
    
    // 速度模式控制 - 包含多机同步标志
    bool Emm_V5_Vel_Control(uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
    
    // 位置模式控制 - 包含绝对/相对模式和多机同步标志
    bool Emm_V5_Pos_Control(uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
    
    // 让电机立即停止运动
    bool Emm_V5_Stop_Now(bool snF);
    
    // 触发多机同步开始运动
    bool Emm_V5_Synchronous_motion();
    
    // 设置挡圈回零的零点位置
    bool Emm_V5_Origin_Set_O(bool svF);
    
    // 修改回零参数
    bool Emm_V5_Origin_Modify_Params(bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, 
                                     uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
    
    // 发送命令触发回零
    bool Emm_V5_Origin_Trigger_Return(uint8_t o_mode, bool snF);
    
    // 强制中断并退出回零
    bool Emm_V5_Origin_Interrupt();
    
    // === 参数读取方法 ===
    bool readMotorStatus(MotorStatus_t& status);
    bool readHomingStatus(HomingStatus_t& status);
    bool readCurrentPosition(float& position_degrees);
    bool readCurrentVelocity(float& velocity_rpm);
    bool readBusVoltage(uint16_t& voltage_mv);
    bool readPhaseCurrent(uint16_t& current_ma);
    
    // === 工具方法 ===
    static float pulsesToDegrees(uint32_t pulses, uint16_t subdivision = 16, float step_angle = 1.8f);
    static uint32_t degreesToPulses(float degrees, uint16_t subdivision = 16, float step_angle = 1.8f);
    
    // 设置电机地址
    void setMotorAddress(uint8_t address) { motor_address_ = address; }
    uint8_t getMotorAddress() const { return motor_address_; }
    
    // 调试功能
    void setDebugMode(bool debug) { debug_mode_ = debug; }
};

#endif // EMM_V5_DRIVER_H
