#include "EmmV5Driver.h"

EmmV5Driver::EmmV5Driver(const std::string& can_interface, uint8_t motor_address) 
    : can_socket_(-1), motor_address_(motor_address), can_interface_(can_interface), debug_mode_(false) {
}

EmmV5Driver::~EmmV5Driver() {
    close();
}

bool EmmV5Driver::initialize() {
    // 创建CAN套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        std::cerr << "Error creating CAN socket: " << strerror(errno) << std::endl;
        return false;
    }
    
    // 设置CAN接口
    struct sockaddr_can addr;
    struct ifreq ifr;
    
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting CAN interface index: " << strerror(errno) << std::endl;
        ::close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding CAN socket: " << strerror(errno) << std::endl;
        ::close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    
    debugPrint("CAN interface " + can_interface_ + " initialized successfully");
    return true;
}

void EmmV5Driver::close() {
    if (can_socket_ >= 0) {
        ::close(can_socket_);
        can_socket_ = -1;
        debugPrint("CAN socket closed");
    }
}

void EmmV5Driver::debugPrint(const std::string& message) {
    if (debug_mode_) {
        std::cout << "[EmmV5Driver] " << message << std::endl;
    }
}

// 基于data.c的can_SendCmd函数实现
void EmmV5Driver::can_SendCmd(const std::vector<uint8_t>& cmd) {
    if (can_socket_ < 0 || cmd.size() < 2) {
        return;
    }
    
    uint8_t txData[8] = {0};
    size_t i = 0;
    size_t j = cmd.size() - 2;  // 计算实际数据长度(除去ID和功能码)
    uint8_t packNum = 0;
    
    while (i < j) {
        size_t k = j - i;
        
        // 配置发送帧头 - 严格按照data.c的实现
        struct can_frame frame;
        frame.can_id = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum | CAN_EFF_FLAG; // 扩展帧
        frame.can_dlc = 0;
        
        // 第一个数据字节为功能码
        txData[0] = cmd[1];
        frame.can_dlc = 1;
        
        // 数据包长度小于8字节
        if (k < 8) {
            for (size_t l = 0; l < k; l++, i++) {
                txData[l + 1] = cmd[i + 2];
            }
            frame.can_dlc = k + 1;
        }
        // 数据包长度大于等于8字节,需要分包发送
        else {
            for (size_t l = 0; l < 7; l++, i++) {
                txData[l + 1] = cmd[i + 2];
            }
            frame.can_dlc = 8;
        }
        
        // 复制数据到帧
        for (size_t idx = 0; idx < frame.can_dlc; idx++) {
            frame.data[idx] = txData[idx];
        }
        
        if (debug_mode_) {
            printf("Send CAN ID: 0x%X, DLC: %d, Data: ", frame.can_id, frame.can_dlc);
            for (size_t idx = 0; idx < frame.can_dlc; idx++) {
                printf("%02X ", frame.data[idx]);
            }
            printf("\n");
        }
        
        // 发送数据
        ssize_t bytes_sent = send(can_socket_, &frame, sizeof(frame), 0);
        if (bytes_sent != sizeof(frame)) {
            std::cerr << "Error sending CAN frame: " << strerror(errno) << std::endl;
            return;
        }
        
        packNum++;  // 包序号增加
        
        // 短暂延时避免总线拥塞
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool EmmV5Driver::receiveCanFrame(std::vector<uint8_t>& data, uint32_t timeout_ms) {
    if (can_socket_ < 0) {
        std::cerr << "CAN socket not initialized" << std::endl;
        return false;
    }
    
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(can_socket_, &readfds);
    
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    
    int ret = select(can_socket_ + 1, &readfds, NULL, NULL, &timeout);
    if (ret <= 0) {
        return false; // 超时或错误
    }
    
    struct can_frame frame;
    ssize_t bytes_received = recv(can_socket_, &frame, sizeof(frame), 0);
    if (bytes_received != sizeof(frame)) {
        return false;
    }
    
    // 检查帧ID是否匹配当前电机地址
    uint8_t frame_address = static_cast<uint8_t>((frame.can_id >> 8) & 0xFF);
    if (frame_address != motor_address_) {
        return false;
    }
    
    data.clear();
    for (size_t i = 0; i < frame.can_dlc; i++) {
        data.push_back(frame.data[i]);
    }
    
    if (debug_mode_) {
        printf("Recv CAN ID: 0x%X, DLC: %d, Data: ", frame.can_id, frame.can_dlc);
        for (size_t i = 0; i < data.size(); i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");
    }
    
    return true;
}

// === 基于原始Emm_V5.c的函数实现 ===

void EmmV5Driver::Emm_V5_Reset_CurPos_To_Zero() {
    std::vector<uint8_t> cmd = {
        motor_address_,     // 地址
        0x0A,              // 功能码
        0x6D,              // 辅助码
        0x6B               // 校验字节
    };
    
    can_SendCmd(cmd);
}

void EmmV5Driver::Emm_V5_Reset_Clog_Pro() {
    std::vector<uint8_t> cmd = {
        motor_address_,     // 地址
        0x0E,              // 功能码
        0x52,              // 辅助码
        0x6B               // 校验字节
    };
    
    can_SendCmd(cmd);
}

void EmmV5Driver::Emm_V5_Read_Sys_Params(SysParams_t s) {
    std::vector<uint8_t> cmd;
    cmd.push_back(motor_address_);  // 地址
    
    switch (s) {
        case S_VER:  cmd.push_back(0x1F); break;
        case S_RL:   cmd.push_back(0x20); break;
        case S_PID:  cmd.push_back(0x21); break;
        case S_VBUS: cmd.push_back(0x24); break;
        case S_CPHA: cmd.push_back(0x27); break;
        case S_ENCL: cmd.push_back(0x31); break;
        case S_TPOS: cmd.push_back(0x33); break;
        case S_VEL:  cmd.push_back(0x35); break;
        case S_CPOS: cmd.push_back(0x36); break;
        case S_PERR: cmd.push_back(0x37); break;
        case S_FLAG: cmd.push_back(0x3A); break;
        case S_ORG:  cmd.push_back(0x3B); break;
        case S_Conf:
            cmd.push_back(0x42);
            cmd.push_back(0x6C);
            break;
        case S_State:
            cmd.push_back(0x43);
            cmd.push_back(0x7A);
            break;
        default:
            return;
    }
    
    cmd.push_back(0x6B);  // 校验字节
    
    can_SendCmd(cmd);
}

void EmmV5Driver::Emm_V5_Modify_Ctrl_Mode(bool svF, uint8_t ctrl_mode) {
    std::vector<uint8_t> cmd = {
        motor_address_,                     // 地址
        0x46,                              // 功能码
        0x69,                              // 辅助码
        static_cast<uint8_t>(svF ? 1 : 0), // 是否存储标志
        ctrl_mode,                         // 控制模式
        0x6B                               // 校验字节
    };
    
    can_SendCmd(cmd);
}

bool EmmV5Driver::Emm_V5_En_Control(bool state, bool snF) {
    std::vector<uint8_t> cmd = {
        motor_address_,                      // 地址
        0xF3,                               // 功能码
        0xAB,                               // 辅助码
        static_cast<uint8_t>(state ? 1 : 0), // 使能状态
        static_cast<uint8_t>(snF ? 1 : 0),   // 多机同步运动标志
        0x6B                                // 校验字节
    };
    
    can_SendCmd(cmd);
    
    // 等待响应
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 1000)) {
        return response.size() >= 3 && response[0] == 0xF3 && response[1] == 0x02;
    }
    
    return false;
}

bool EmmV5Driver::Emm_V5_Vel_Control(uint8_t dir, uint16_t vel, uint8_t acc, bool snF) {
    std::vector<uint8_t> cmd = {
        motor_address_,                      // 地址
        0xF6,                               // 功能码
        dir,                                // 方向
        static_cast<uint8_t>(vel >> 8),     // 速度(RPM)高8位字节
        static_cast<uint8_t>(vel & 0xFF),   // 速度(RPM)低8位字节
        acc,                                // 加速度，注意：0是直接启动
        static_cast<uint8_t>(snF ? 1 : 0),  // 多机同步运动标志
        0x6B                                // 校验字节
    };
    
    can_SendCmd(cmd);
    
    // 等待响应
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 1000)) {
        return response.size() >= 3 && response[0] == 0xF6 && response[1] == 0x02;
    }
    
    return false;
}

bool EmmV5Driver::Emm_V5_Pos_Control(uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF) {
    std::vector<uint8_t> cmd = {
        motor_address_,                      // 地址
        0xFD,                               // 功能码
        dir,                                // 方向
        static_cast<uint8_t>(vel >> 8),     // 速度(RPM)高8位字节
        static_cast<uint8_t>(vel & 0xFF),   // 速度(RPM)低8位字节
        acc,                                // 加速度，注意：0是直接启动
        static_cast<uint8_t>(clk >> 24),    // 脉冲数(bit24 - bit31)
        static_cast<uint8_t>(clk >> 16),    // 脉冲数(bit16 - bit23)
        static_cast<uint8_t>(clk >> 8),     // 脉冲数(bit8  - bit15)
        static_cast<uint8_t>(clk & 0xFF),   // 脉冲数(bit0  - bit7 )
        static_cast<uint8_t>(raF ? 1 : 0),  // 相位/绝对标志，false为相对运动，true为绝对值运动
        static_cast<uint8_t>(snF ? 1 : 0),  // 多机同步运动标志，false为不启用，true为启用
        0x6B                                // 校验字节
    };
    
    can_SendCmd(cmd);
    
    // 等待响应
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 1000)) {
        return response.size() >= 3 && response[0] == 0xFD && response[1] == 0x02;
    }
    
    return false;
}

bool EmmV5Driver::Emm_V5_Stop_Now(bool snF) {
    std::vector<uint8_t> cmd = {
        motor_address_,                      // 地址
        0xFE,                               // 功能码
        0x98,                               // 辅助码
        static_cast<uint8_t>(snF ? 1 : 0),  // 多机同步运动标志
        0x6B                                // 校验字节
    };
    
    can_SendCmd(cmd);
    
    // 等待响应
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 1000)) {
        return response.size() >= 3 && response[0] == 0xFE && response[1] == 0x02;
    }
    
    return false;
}

bool EmmV5Driver::Emm_V5_Synchronous_motion() {
    std::vector<uint8_t> cmd = {
        motor_address_,     // 地址
        0xFF,              // 功能码
        0x66,              // 辅助码
        0x6B               // 校验字节
    };
    
    can_SendCmd(cmd);
    
    // 等待响应
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 1000)) {
        return response.size() >= 3 && response[0] == 0xFF && response[1] == 0x02;
    }
    
    return false;
}

bool EmmV5Driver::Emm_V5_Origin_Set_O(bool svF) {
    std::vector<uint8_t> cmd = {
        motor_address_,                      // 地址
        0x93,                               // 功能码
        0x88,                               // 辅助码
        static_cast<uint8_t>(svF ? 1 : 0),  // 是否存储标志
        0x6B                                // 校验字节
    };
    
    can_SendCmd(cmd);
    
    // 等待响应
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 1000)) {
        return response.size() >= 3 && response[0] == 0x93 && response[1] == 0x02;
    }
    
    return false;
}

bool EmmV5Driver::Emm_V5_Origin_Modify_Params(bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel,
                                              uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF) {
    std::vector<uint8_t> cmd = {
        motor_address_,                      // 地址
        0x4C,                               // 功能码
        0xAE,                               // 辅助码
        static_cast<uint8_t>(svF ? 1 : 0),  // 是否存储标志
        o_mode,                             // 回零模式
        o_dir,                              // 回零方向
        static_cast<uint8_t>(o_vel >> 8),   // 回零速度(RPM)高8位字节
        static_cast<uint8_t>(o_vel & 0xFF), // 回零速度(RPM)低8位字节
        static_cast<uint8_t>(o_tm >> 24),   // 回零超时时间(bit24 - bit31)
        static_cast<uint8_t>(o_tm >> 16),   // 回零超时时间(bit16 - bit23)
        static_cast<uint8_t>(o_tm >> 8),    // 回零超时时间(bit8  - bit15)
        static_cast<uint8_t>(o_tm & 0xFF),  // 回零超时时间(bit0  - bit7 )
        static_cast<uint8_t>(sl_vel >> 8),  // 无限位碰撞回零检测转速(RPM)高8位字节
        static_cast<uint8_t>(sl_vel & 0xFF), // 无限位碰撞回零检测转速(RPM)低8位字节
        static_cast<uint8_t>(sl_ma >> 8),   // 无限位碰撞回零检测电流(Ma)高8位字节
        static_cast<uint8_t>(sl_ma & 0xFF), // 无限位碰撞回零检测电流(Ma)低8位字节
        static_cast<uint8_t>(sl_ms >> 8),   // 无限位碰撞回零检测时间(Ms)高8位字节
        static_cast<uint8_t>(sl_ms & 0xFF), // 无限位碰撞回零检测时间(Ms)低8位字节
        static_cast<uint8_t>(potF ? 1 : 0), // 上电自动触发回零
        0x6B                                // 校验字节
    };
    
    can_SendCmd(cmd);
    
    // 等待响应
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 2000)) {
        return response.size() >= 3 && response[0] == 0x4C && response[1] == 0x02;
    }
    
    return false;
}

bool EmmV5Driver::Emm_V5_Origin_Trigger_Return(uint8_t o_mode, bool snF) {
    std::vector<uint8_t> cmd = {
        motor_address_,                      // 地址
        0x9A,                               // 功能码
        o_mode,                             // 回零模式
        static_cast<uint8_t>(snF ? 1 : 0),  // 多机同步运动标志
        0x6B                                // 校验字节
    };
    
    can_SendCmd(cmd);
    
    // 等待响应
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 1000)) {
        return response.size() >= 3 && response[0] == 0x9A && response[1] == 0x02;
    }
    
    return false;
}

bool EmmV5Driver::Emm_V5_Origin_Interrupt() {
    std::vector<uint8_t> cmd = {
        motor_address_,     // 地址
        0x9C,              // 功能码
        0x48,              // 辅助码
        0x6B               // 校验字节
    };
    
    can_SendCmd(cmd);
    
    // 等待响应
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 1000)) {
        return response.size() >= 3 && response[0] == 0x9C && response[1] == 0x02;
    }
    
    return false;
}

// === 参数读取方法实现 ===
bool EmmV5Driver::readMotorStatus(MotorStatus_t& status) {
    Emm_V5_Read_Sys_Params(S_FLAG);
    
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 2000)) {
        if (response.size() >= 3 && response[0] == 0x3A) {
            uint8_t status_byte = response[1];
            status.enabled = (status_byte & 0x01) != 0;
            status.in_position = (status_byte & 0x02) != 0;
            status.stalled = (status_byte & 0x04) != 0;
            status.stall_protection = (status_byte & 0x08) != 0;
            return true;
        }
    }
    
    return false;
}

bool EmmV5Driver::readHomingStatus(HomingStatus_t& status) {
    Emm_V5_Read_Sys_Params(S_ORG);
    
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 2000)) {
        if (response.size() >= 3 && response[0] == 0x3B) {
            uint8_t status_byte = response[1];
            status.encoder_ready = (status_byte & 0x01) != 0;
            status.calibration_ready = (status_byte & 0x02) != 0;
            status.homing = (status_byte & 0x04) != 0;
            status.homing_failed = (status_byte & 0x08) != 0;
            return true;
        }
    }
    
    return false;
}

bool EmmV5Driver::readCurrentPosition(float& position_degrees) {
    Emm_V5_Read_Sys_Params(S_CPOS);
    
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 2000)) {
        if (response.size() >= 6 && response[0] == 0x36) {
            uint8_t sign = response[1];
            uint32_t position_raw = (static_cast<uint32_t>(response[2]) << 24) |
                                   (static_cast<uint32_t>(response[3]) << 16) |
                                   (static_cast<uint32_t>(response[4]) << 8) |
                                    static_cast<uint32_t>(response[5]);
            
            position_degrees = (position_raw * 360.0f) / 65536.0f;
            if (sign == 1) {
                position_degrees = -position_degrees;
            }
            return true;
        }
    }
    
    return false;
}

bool EmmV5Driver::readCurrentVelocity(float& velocity_rpm) {
    Emm_V5_Read_Sys_Params(S_VEL);
    
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 2000)) {
        if (response.size() >= 4 && response[0] == 0x35) {
            uint8_t sign = response[1];
            uint16_t velocity_raw = (static_cast<uint16_t>(response[2]) << 8) |
                                    static_cast<uint16_t>(response[3]);
            
            velocity_rpm = static_cast<float>(velocity_raw);
            if (sign == 1) {
                velocity_rpm = -velocity_rpm;
            }
            return true;
        }
    }
    
    return false;
}

bool EmmV5Driver::readBusVoltage(uint16_t& voltage_mv) {
    Emm_V5_Read_Sys_Params(S_VBUS);
    
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 2000)) {
        if (response.size() >= 4 && response[0] == 0x24) {
            voltage_mv = (static_cast<uint16_t>(response[1]) << 8) |
                         static_cast<uint16_t>(response[2]);
            return true;
        }
    }
    
    return false;
}

bool EmmV5Driver::readPhaseCurrent(uint16_t& current_ma) {
    Emm_V5_Read_Sys_Params(S_CPHA);
    
    std::vector<uint8_t> response;
    if (receiveCanFrame(response, 2000)) {
        if (response.size() >= 4 && response[0] == 0x27) {
            current_ma = (static_cast<uint16_t>(response[1]) << 8) |
                         static_cast<uint16_t>(response[2]);
            return true;
        }
    }
    
    return false;
}

// === 工具方法实现 ===
float EmmV5Driver::pulsesToDegrees(uint32_t pulses, uint16_t subdivision, float step_angle) {
    return (static_cast<float>(pulses) * step_angle) / subdivision;
}

uint32_t EmmV5Driver::degreesToPulses(float degrees, uint16_t subdivision, float step_angle) {
    return static_cast<uint32_t>((degrees * subdivision) / step_angle);
}
