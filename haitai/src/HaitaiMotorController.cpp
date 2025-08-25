// HaitaiMotorController.cpp
#include "HaitaiMotorController.h"
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>

// Unix/Linux平台特定头文件
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <dirent.h>

// 定义静态常量(这一行修复了链接错误)
const uint8_t HaitaiMotorController::HEADER_HOST = 0x3E;
const uint8_t HaitaiMotorController::HEADER_SLAVE = 0x3C;

// CRC16 MODBUS查表实现
const uint16_t HaitaiMotorController::CRC16_TABLE[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

// Linux优化串口类实现
class HaitaiMotorController::SerialPort {
public:
    SerialPort(const std::string& port, int baudrate) : 
        port_(port), baudrate_(baudrate), isOpen_(false), fd_(-1) {
    }
    
    ~SerialPort() {
        close();
    }
    
    bool open() {
        if (isOpen_) return true;
        
        // 打开串口
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        
        if (fd_ < 0) {
            lastError_ = "打开串口失败: " + std::string(strerror(errno));
            return false;
        }
        
        struct termios options;
        if (tcgetattr(fd_, &options) != 0) {
            lastError_ = "获取串口配置失败: " + std::string(strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        // 设置波特率
        speed_t brate;
        switch (baudrate_) {
            case 9600:   brate = B9600;   break;
            case 19200:  brate = B19200;  break;
            case 38400:  brate = B38400;  break;
            case 57600:  brate = B57600;  break;
            case 115200: brate = B115200; break;
            default:     brate = B115200; break;
        }
        
        cfsetispeed(&options, brate);
        cfsetospeed(&options, brate);
        
        // 控制模式设置
        options.c_cflag |= (CLOCAL | CREAD); // 忽略控制线，启用接收
        options.c_cflag &= ~PARENB;          // 无奇偶校验
        options.c_cflag &= ~CSTOPB;          // 1个停止位
        options.c_cflag &= ~CSIZE;           // 清除数据位掩码
        options.c_cflag |= CS8;              // 8个数据位
        
        // 设置为原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 禁用规范模式处理
        options.c_oflag &= ~OPOST;                          // 禁用输出处理
        
        // 输入模式设置：禁用软件流控制，使字符不经处理立即可用
        options.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        // 设置读取超时
        options.c_cc[VMIN] = 0;  // 非规范模式下读取的最小字符数
        options.c_cc[VTIME] = 0; // 非规范模式下等待数据的时间（单位为0.1秒）
        
        // 应用设置
        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            lastError_ = "设置串口参数失败: " + std::string(strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        // 刷新缓冲区
        tcflush(fd_, TCIOFLUSH);
        
        isOpen_ = true;
        return true;
    }
    
    void close() {
        if (isOpen_ && fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
            isOpen_ = false;
        }
    }
    
    bool isOpen() const {
        return isOpen_;
    }
    
    int write(const std::vector<uint8_t>& data) {
        if (!isOpen_ || fd_ < 0) {
            lastError_ = "串口未打开";
            return -1;
        }
        
        ssize_t bytesWritten = ::write(fd_, data.data(), data.size());
        if (bytesWritten < 0) {
            lastError_ = "写入串口失败: " + std::string(strerror(errno));
            return -1;
        }
        
        return static_cast<int>(bytesWritten);
    }
    
    std::vector<uint8_t> read(size_t maxBytes, unsigned int timeout) {
        std::vector<uint8_t> result;
        
        if (!isOpen_ || fd_ < 0) {
            lastError_ = "串口未打开";
            return result;
        }
        
        // 使用select实现超时
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);
        
        struct timeval tv;
        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        
        int ret = select(fd_ + 1, &readfds, NULL, NULL, &tv);
        
        if (ret < 0) {
            lastError_ = "select失败: " + std::string(strerror(errno));
            return result;
        } else if (ret == 0) {
            // 超时
            return result;
        }
        
        // 有数据可读
        std::vector<uint8_t> buffer(maxBytes);
        ssize_t bytesRead = ::read(fd_, buffer.data(), maxBytes);
        
        if (bytesRead < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 非阻塞模式下没有数据
                return result;
            }
            lastError_ = "读取串口失败: " + std::string(strerror(errno));
            return result;
        }
        
        buffer.resize(bytesRead);
        return buffer;
    }
    
    std::vector<uint8_t> readExact(size_t numBytes, unsigned int timeout) {
        std::vector<uint8_t> result;
        result.reserve(numBytes);
        
        if (!isOpen_ || fd_ < 0) {
            lastError_ = "串口未打开";
            return result;
        }
        
        auto startTime = std::chrono::steady_clock::now();
        auto endTime = startTime + std::chrono::milliseconds(timeout);
        
        while (result.size() < numBytes) {
            auto now = std::chrono::steady_clock::now();
            if (now >= endTime) break;
            
            auto remainingTimeout = std::chrono::duration_cast<std::chrono::milliseconds>(
                endTime - now).count();
            
            std::vector<uint8_t> chunk = read(numBytes - result.size(), 
                                           static_cast<unsigned int>(remainingTimeout));
            
            if (chunk.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            
            result.insert(result.end(), chunk.begin(), chunk.end());
        }
        
        return result;
    }
    
    void flush() {
        if (isOpen_ && fd_ >= 0) {
            tcflush(fd_, TCIOFLUSH);
        }
    }
    
    int available() {
        if (!isOpen_ || fd_ < 0) {
            return 0;
        }
        
        int bytes = 0;
        if (ioctl(fd_, FIONREAD, &bytes) < 0) {
            return 0;
        }
        return bytes;
    }
    
    std::string getLastError() const {
        return lastError_;
    }
    
private:
    std::string port_;
    int baudrate_;
    bool isOpen_;
    int fd_;
    std::string lastError_;
};

//-----------------------------------------------
// HaitaiMotorController实现
//-----------------------------------------------

HaitaiMotorController::HaitaiMotorController(
    const std::string& port,
    int baudrate,
    const std::map<std::string, uint8_t>& motorAddresses,
    uint8_t defaultAddress,
    unsigned int timeout,
    unsigned int retries
) : port_(port),
    baudrate_(baudrate),
    timeout_(timeout),
    retries_(retries),
    monitoringActive_(false) {
    
    // 创建串口实例
    serial_ = std::make_unique<SerialPort>(port, baudrate);
    
    // 初始化电机地址
    if (motorAddresses.empty()) {
        defaultMotorId_ = "default";
        
        MotorState state;
        state.address = defaultAddress;
        state.limits = {-1000000.0f, 1000000.0f, 3000.0f};  // 默认限制
        
        motors_[defaultMotorId_] = state;
    } else {
        defaultMotorId_ = motorAddresses.begin()->first;
        
        for (const auto& pair : motorAddresses) {
            MotorState state;
            state.address = pair.second;
            state.limits = {-1000000.0f, 1000000.0f, 3000.0f};  // 默认限制
            
            motors_[pair.first] = state;
            
            // 初始化控制参数
            controlParams_.pdParams[pair.first] = ControlParameters::PDParams();
            controlParams_.trajectoryParams[pair.first] = ControlParameters::TrajectoryParams();
        }
    }
    
    // 初始化默认参数
    controlParams_.speedPID = ControlParameters::SpeedPIDParams();
    controlParams_.safety = ControlParameters::SafetyParams();
}

HaitaiMotorController::~HaitaiMotorController() {
    // 停止监控线程
    stopStatusMonitoring();
    
    // 尝试关闭所有电机
    try {
        shutdownAllMotors();
    } catch (...) {
        // 忽略关闭过程中的异常
    }
    
    // 关闭串口
    close();
}

bool HaitaiMotorController::open() {
    std::lock_guard<std::mutex> lock(serialMutex_);
    return serial_ && serial_->open();
}

void HaitaiMotorController::close() {
    std::lock_guard<std::mutex> lock(serialMutex_);
    if (serial_) {
        serial_->close();
    }
}

bool HaitaiMotorController::isOpen() const {
    return serial_ && serial_->isOpen();
}

bool HaitaiMotorController::addMotor(const std::string& motorId, uint8_t address) {
    if (motorId.empty()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(motorsMutex_);
    
    // 检查是否已存在
    if (motors_.find(motorId) != motors_.end()) {
        return false;
    }
    
    // 添加新电机
    MotorState state;
    state.address = address;
    state.limits = {-1000000.0f, 1000000.0f, 3000.0f};  // 默认限制
    
    motors_[motorId] = state;
    
    // 添加控制参数
    controlParams_.pdParams[motorId] = ControlParameters::PDParams();
    controlParams_.trajectoryParams[motorId] = ControlParameters::TrajectoryParams();
    
    return true;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::setMotorLimits(
    const std::string& motorId, const MotorLimits& limits) {
    
    std::lock_guard<std::mutex> lock(motorsMutex_);
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    
    auto it = motors_.find(id);
    if (it == motors_.end()) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, "未知的电机ID");
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    // 验证限制参数
    if (limits.minAngle >= limits.maxAngle) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, "无效的角度限制范围");
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    if (limits.maxSpeed <= 0) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, "速度限制必须为正值");
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    // 设置限制
    it->second.limits = limits;
    
    return ErrorCode::SUCCESS;
}

bool HaitaiMotorController::setPDParams(const std::string& motorId, float kp, float kd) {
    std::lock_guard<std::mutex> lock(motorsMutex_);  // 现在可以正常使用
    
    if (motorId.empty() || motorId == "all") {
        // 设置所有电机
        for (auto& pair : motors_) {
            controlParams_.pdParams[pair.first] = {kp, kd};
        }
        return true;
    } else {
        // 设置指定电机
        auto it = motors_.find(motorId);
        if (it == motors_.end()) {
            if (errorCallback_) {
                errorCallback_(motorId, ErrorCode::PARAMETER_ERROR, "未知的电机ID");
            }
            return false;
        }
        
        controlParams_.pdParams[motorId] = {kp, kd};
        return true;
    }
}

HaitaiMotorController::ErrorCode HaitaiMotorController::setSpeedPIDParams(
    const std::string& motorId, const ControlParameters::SpeedPIDParams& params) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    
    try {
        // 读取当前参数
        SystemParameters sysParams;
        ErrorCode result = readSystemParams(id, sysParams);
        if (result != ErrorCode::SUCCESS) {
            return result;
        }
        
        // 更新参数
        sysParams.speedKp = params.kp;
        sysParams.speedKi = params.ki;
        sysParams.speedFilter = params.filter;
        
        // 写入参数
        result = writeSystemParamsTemp(id, sysParams);
        if (result == ErrorCode::SUCCESS) {
            // 更新内部记录
            controlParams_.speedPID = params;
        }
        
        return result;
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::UNKNOWN_ERROR, 
                         std::string("设置速度PID参数失败: ") + e.what());
        }
        return ErrorCode::UNKNOWN_ERROR;
    }
}

void HaitaiMotorController::loadParameters(const ControlParameters& params) {
    std::lock_guard<std::mutex> lock(motorsMutex_);
    controlParams_ = params;
    
    // 更新电机参数
    for (const auto& pair : controlParams_.pdParams) {
        if (motors_.find(pair.first) != motors_.end()) {
            // 记录PD参数
        }
    }
    
    // 设置安全参数限制
    for (auto& pair : motors_) {
        pair.second.limits.maxSpeed = params.safety.maxSpeed;
    }
}

HaitaiMotorController::ControlParameters HaitaiMotorController::getCurrentParameters() const {
    std::lock_guard<std::mutex> lock(motorsMutex_);
    return controlParams_;
}

bool HaitaiMotorController::saveParametersToFile(const std::string& filename) const {
    try {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        // 写入PD参数
        file << "[PD_PARAMS]" << std::endl;
        for (const auto& pair : controlParams_.pdParams) {
            file << pair.first << ".kp=" << pair.second.kp << std::endl;
            file << pair.first << ".kd=" << pair.second.kd << std::endl;
        }
        
        // 写入速度PID参数
        file << "[SPEED_PID]" << std::endl;
        file << "kp=" << controlParams_.speedPID.kp << std::endl;
        file << "ki=" << controlParams_.speedPID.ki << std::endl;
        file << "filter=" << controlParams_.speedPID.filter << std::endl;
        
        // 写入轨迹参数
        file << "[TRAJECTORY]" << std::endl;
        for (const auto& pair : controlParams_.trajectoryParams) {
            file << pair.first << ".amplitude=" << pair.second.amplitude << std::endl;
            file << pair.first << ".frequency=" << pair.second.frequency << std::endl;
            file << pair.first << ".phase=" << pair.second.phase << std::endl;
        }
        
        // 写入安全参数
        file << "[SAFETY]" << std::endl;
        file << "max_position_error=" << controlParams_.safety.maxPositionError << std::endl;
        file << "error_timeout=" << controlParams_.safety.errorTimeout << std::endl;
        file << "max_speed=" << controlParams_.safety.maxSpeed << std::endl;
        
        // 写入控制参数
        file << "[CONTROL]" << std::endl;
        file << "frequency=" << controlParams_.controlFrequency << std::endl;
        file << "duration=" << controlParams_.duration << std::endl;
        
        file.close();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool HaitaiMotorController::loadParametersFromFile(const std::string& filename) {
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        ControlParameters params;
        
        // 默认参数
        params.speedPID = {0.8f, 0.2f, 0.9f};
        params.safety = {25.0f, 2.0f, 3000.0f};
        params.controlFrequency = 100.0f;
        params.duration = 15.0f;
        
        std::string line, section;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue; // 跳过空行和注释
            }
            
            if (line[0] == '[' && line.back() == ']') {
                section = line.substr(1, line.size() - 2);
                continue;
            }
            
            size_t pos = line.find('=');
            if (pos == std::string::npos) {
                continue;
            }
            
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);
            float fvalue = std::stof(value);
            
            if (section == "PD_PARAMS") {
                size_t dotPos = key.find('.');
                if (dotPos != std::string::npos) {
                    std::string motorId = key.substr(0, dotPos);
                    std::string param = key.substr(dotPos + 1);
                    
                    if (param == "kp") {
                        params.pdParams[motorId].kp = fvalue;
                    } else if (param == "kd") {
                        params.pdParams[motorId].kd = fvalue;
                    }
                }
            } else if (section == "SPEED_PID") {
                if (key == "kp") {
                    params.speedPID.kp = fvalue;
                } else if (key == "ki") {
                    params.speedPID.ki = fvalue;
                } else if (key == "filter") {
                    params.speedPID.filter = fvalue;
                }
            } else if (section == "TRAJECTORY") {
                size_t dotPos = key.find('.');
                if (dotPos != std::string::npos) {
                    std::string motorId = key.substr(0, dotPos);
                    std::string param = key.substr(dotPos + 1);
                    
                    if (param == "amplitude") {
                        params.trajectoryParams[motorId].amplitude = fvalue;
                    } else if (param == "frequency") {
                        params.trajectoryParams[motorId].frequency = fvalue;
                    } else if (param == "phase") {
                        params.trajectoryParams[motorId].phase = fvalue;
                    }
                }
            } else if (section == "SAFETY") {
                if (key == "max_position_error") {
                    params.safety.maxPositionError = fvalue;
                } else if (key == "error_timeout") {
                    params.safety.errorTimeout = fvalue;
                } else if (key == "max_speed") {
                    params.safety.maxSpeed = fvalue;
                }
            } else if (section == "CONTROL") {
                if (key == "frequency") {
                    params.controlFrequency = fvalue;
                } else if (key == "duration") {
                    params.duration = fvalue;
                }
            }
        }
        
        file.close();
        
        // 加载参数
        loadParameters(params);
        
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

uint8_t HaitaiMotorController::getAddress(const std::string& motorId) {
    std::lock_guard<std::mutex> lock(motorsMutex_);
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    
    auto it = motors_.find(id);
    if (it == motors_.end()) {
        throw std::runtime_error("未知的电机ID: " + id);
    }
    
    return it->second.address;
}

uint8_t HaitaiMotorController::nextSequence(const std::string& motorId) {
    std::lock_guard<std::mutex> lock(motorsMutex_);
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    
    auto it = motors_.find(id);
    if (it == motors_.end()) {
        throw std::runtime_error("未知的电机ID: " + id);
    }
    
    uint8_t seq = it->second.packetSeq;
    it->second.packetSeq = (it->second.packetSeq + 1) % 256;
    
    return seq;
}

std::vector<uint8_t> HaitaiMotorController::buildPacket(
    uint8_t cmd, uint8_t address, const std::vector<uint8_t>& data, const std::string& motorId) {
    
    uint8_t seq = nextSequence(motorId);
    uint8_t len = static_cast<uint8_t>(data.size());
    
    // 构建头部
    std::vector<uint8_t> packet;
    packet.push_back(HEADER_HOST);
    packet.push_back(seq);
    packet.push_back(address);
    packet.push_back(cmd);
    packet.push_back(len);
    
    // 添加数据
    packet.insert(packet.end(), data.begin(), data.end());
    
    // 计算CRC
    uint16_t crc = calculateCRC(packet);
    
    // 添加CRC (低字节在前)
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);
    
    return packet;
}

uint16_t HaitaiMotorController::calculateCRC(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t byte : data) {
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF];
    }
    
    return crc;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::sendAndReceive(
    uint8_t cmd, uint8_t address, const std::vector<uint8_t>& sendData, 
    std::vector<uint8_t>& receiveData, const std::string& motorId,
    unsigned int expectedResponseLen) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    
    std::lock_guard<std::mutex> lock(serialMutex_);
    
    if (!serial_ || !serial_->isOpen()) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::SERIAL_ERROR, "串口未打开");
        }
        return ErrorCode::SERIAL_ERROR;
    }
    
    // 构建数据包
    std::vector<uint8_t> packet;
    try {
        packet = buildPacket(cmd, address, sendData, id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, std::string("构建数据包失败: ") + e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    // 实现通信重试机制
    for (unsigned int attempt = 0; attempt <= retries_; ++attempt) {
        // 清空输入缓冲区
        serial_->flush();
        
        // 发送数据包
        int bytesWritten = serial_->write(packet);
        if (bytesWritten != static_cast<int>(packet.size())) {
            if (attempt == retries_) {
                if (errorCallback_) {
                    errorCallback_(id, ErrorCode::SERIAL_ERROR, 
                                 "数据包写入不完整: " + serial_->getLastError());
                }
                return ErrorCode::SERIAL_ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        
        // 读取响应头部(5字节)
        std::vector<uint8_t> header = serial_->readExact(5, timeout_);
        if (header.size() < 5) {
            if (attempt == retries_) {
                if (errorCallback_) {
                    errorCallback_(id, ErrorCode::TIMEOUT_ERROR, 
                                 "等待响应头部超时");
                }
                return ErrorCode::TIMEOUT_ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        
        // 解析头部
        uint8_t responseHeader = header[0];
        uint8_t responseSeq = header[1];
        uint8_t responseAddr = header[2];
        uint8_t responseCmd = header[3];
        uint8_t responseLen = header[4];
        
        // 验证头部
        if (responseHeader != HEADER_SLAVE || responseAddr != address || responseCmd != cmd) {
            if (attempt == retries_) {
                std::stringstream ss;
                ss << "响应头部不匹配: 期望 [Header=" << std::hex << static_cast<int>(HEADER_SLAVE)
                   << ", Addr=" << static_cast<int>(address) << ", Cmd=" << static_cast<int>(cmd)
                   << "], 实际 [Header=" << static_cast<int>(responseHeader)
                   << ", Addr=" << static_cast<int>(responseAddr)
                   << ", Cmd=" << static_cast<int>(responseCmd) << "]";
                if (errorCallback_) {
                    errorCallback_(id, ErrorCode::RESPONSE_ERROR, ss.str());
                }
                return ErrorCode::RESPONSE_ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        
        // 检查期望的响应长度
        if (expectedResponseLen > 0 && responseLen != expectedResponseLen) {
            if (attempt == retries_) {
                std::stringstream ss;
                ss << "响应长度不匹配: 期望 " << static_cast<int>(expectedResponseLen)
                   << ", 实际 " << static_cast<int>(responseLen);
                if (errorCallback_) {
                    errorCallback_(id, ErrorCode::RESPONSE_ERROR, ss.str());
                }
                return ErrorCode::RESPONSE_ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        
        // 读取数据和CRC
        std::vector<uint8_t> dataAndCrc = serial_->readExact(responseLen + 2, timeout_);
        if (dataAndCrc.size() < responseLen + 2) {
            if (attempt == retries_) {
                if (errorCallback_) {
                    errorCallback_(id, ErrorCode::TIMEOUT_ERROR,
                                 "等待响应数据超时");
                }
                return ErrorCode::TIMEOUT_ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        
        // 提取数据和CRC
        std::vector<uint8_t> responseData(dataAndCrc.begin(), dataAndCrc.begin() + responseLen);
        uint16_t receivedCrc = (static_cast<uint16_t>(dataAndCrc[responseLen + 1]) << 8) | 
                               dataAndCrc[responseLen];
        
        // 验证CRC
        std::vector<uint8_t> fullPacket = header;
        fullPacket.insert(fullPacket.end(), responseData.begin(), responseData.end());
        uint16_t calculatedCrc = calculateCRC(fullPacket);
        
        if (receivedCrc != calculatedCrc) {
            if (attempt == retries_) {
                std::stringstream ss;
                ss << "CRC校验失败: 期望 0x" << std::hex << calculatedCrc
                   << ", 实际 0x" << receivedCrc;
                if (errorCallback_) {
                    errorCallback_(id, ErrorCode::CRC_ERROR, ss.str());
                }
                return ErrorCode::CRC_ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        
        // 成功 - 设置输出数据
        receiveData = responseData;
        return ErrorCode::SUCCESS;
    }
    
    // 所有重试都失败
    if (errorCallback_) {
        errorCallback_(id, ErrorCode::COMMUNICATION_ERROR, "通信失败，已达到最大重试次数");
    }
    return ErrorCode::COMMUNICATION_ERROR;
}

bool HaitaiMotorController::validateMotionLimits(
    const std::string& motorId, float targetPosition, float targetSpeed) {
    
    std::lock_guard<std::mutex> lock(motorsMutex_);
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    
    auto it = motors_.find(id);
    if (it == motors_.end()) {
        return false;
    }
    
    const MotorLimits& limits = it->second.limits;
    
    // 检查位置限制
    if (targetPosition < limits.minAngle || targetPosition > limits.maxAngle) {
        if (errorCallback_) {
            std::stringstream ss;
            ss << "目标位置 " << targetPosition << "° 超出限制范围 ["
               << limits.minAngle << "°, " << limits.maxAngle << "°]";
            errorCallback_(id, ErrorCode::MOTION_LIMIT_ERROR, ss.str());
        }
        return false;
    }
    
    // 检查速度限制
    if (std::abs(targetSpeed) > limits.maxSpeed) {
        if (errorCallback_) {
            std::stringstream ss;
            ss << "目标速度 " << std::abs(targetSpeed) << " RPM 超出限制 "
               << limits.maxSpeed << " RPM";
            errorCallback_(id, ErrorCode::MOTION_LIMIT_ERROR, ss.str());
        }
        return false;
    }
    
    return true;
}

void HaitaiMotorController::monitoringFunction(unsigned int interval) {
    while (monitoringActive_) {
        // 复制电机列表以避免锁定问题
        std::vector<std::string> motorIds;
        {
            std::lock_guard<std::mutex> lock(motorsMutex_);
            for (const auto& pair : motors_) {
                motorIds.push_back(pair.first);
            }
        }
        
        // 读取每个电机的状态
        for (const std::string& id : motorIds) {
            if (!monitoringActive_) {
                break;
            }
            
            try {
                MotorStatus status;
                if (readMotorStatus(id, status) == ErrorCode::SUCCESS && statusCallback_) {
                    statusCallback_(id, status);
                    
                    // 更新内部状态
                    std::lock_guard<std::mutex> lock(motorsMutex_);
                    auto it = motors_.find(id);
                    if (it != motors_.end()) {
                        it->second.status = status;
                        it->second.lastUpdateTime = std::chrono::steady_clock::now();
                    }
                }
            } catch (const std::exception&) {
                // 忽略监控过程中的异常
            }
            
            // 在电机之间添加小延迟，避免总线拥堵
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        // 等待下一个周期
        std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
}

bool HaitaiMotorController::startStatusMonitoring(
    unsigned int interval, StatusCallback callback) {
    
    // 避免重复启动
    if (monitoringActive_.load()) {
        stopStatusMonitoring();
    }
    
    statusCallback_ = callback;
    monitoringActive_.store(true);
    
    try {
        monitoringThread_ = std::make_unique<std::thread>(
            &HaitaiMotorController::monitoringFunction, this, interval
        );
        return true;
    } catch (const std::exception&) {
        monitoringActive_.store(false);
        return false;
    }
}

void HaitaiMotorController::stopStatusMonitoring() {
    // 设置停止标志
    monitoringActive_.store(false);
    
    // 等待线程结束
    if (monitoringThread_ && monitoringThread_->joinable()) {
        monitoringThread_->join();
        monitoringThread_.reset();
    }
}

void HaitaiMotorController::setErrorCallback(ErrorCallback callback) {
    errorCallback_ = callback;
}

std::string HaitaiMotorController::getErrorMessage(ErrorCode code) {
    switch (code) {
        case ErrorCode::SUCCESS:           return "成功";
        case ErrorCode::SERIAL_ERROR:      return "串口通信错误";
        case ErrorCode::TIMEOUT_ERROR:     return "通信超时";
        case ErrorCode::CRC_ERROR:         return "CRC校验错误";
        case ErrorCode::RESPONSE_ERROR:    return "响应格式错误";
        case ErrorCode::PARAMETER_ERROR:   return "参数错误";
        case ErrorCode::MOTION_LIMIT_ERROR: return "运动超出限制";
        case ErrorCode::COMMUNICATION_ERROR: return "通信错误";
        case ErrorCode::UNKNOWN_ERROR:     return "未知错误";
        default:                         return "未定义错误";
    }
}

// 参数解析和打包实现
HaitaiMotorController::SystemParameters HaitaiMotorController::parseSystemParameters(
    const std::vector<uint8_t>& data) {
    
    SystemParameters params;
    
    params.deviceAddress = data[0];
    params.currentThreshold = data[1] * 0.03f;
    params.voltageThreshold = data[2] * 0.2f;
    params.baudrates = data[3];
    
    // 解析浮点数参数
    std::memcpy(&params.positionKp, &data[4], 4);
    std::memcpy(&params.positionTargetSpeed, &data[8], 4);
    params.positionTargetSpeed *= 0.1f;
    std::memcpy(&params.speedKp, &data[12], 4);
    std::memcpy(&params.speedKi, &data[16], 4);
    std::memcpy(&params.reserved, &data[20], 4);
    
    params.speedFilter = data[24] / 100.0f;
    params.powerPercent = data[25];
    
    return params;
}

std::vector<uint8_t> HaitaiMotorController::packSystemParameters(
    const SystemParameters& params) {
    
    std::vector<uint8_t> data(26);
    
    data[0] = params.deviceAddress;
    data[1] = static_cast<uint8_t>(params.currentThreshold / 0.03f);
    data[2] = static_cast<uint8_t>(params.voltageThreshold / 0.2f);
    data[3] = params.baudrates;
    
    // 打包浮点数参数
    float positionTargetSpeedScaled = params.positionTargetSpeed / 0.1f;
    
    std::memcpy(&data[4], &params.positionKp, 4);
    std::memcpy(&data[8], &positionTargetSpeedScaled, 4);
    std::memcpy(&data[12], &params.speedKp, 4);
    std::memcpy(&data[16], &params.speedKi, 4);
    std::memcpy(&data[20], &params.reserved, 4);
    
    data[24] = static_cast<uint8_t>(params.speedFilter * 100.0f);
    data[25] = params.powerPercent;
    
    return data;
}

// 具体命令实现
HaitaiMotorController::ErrorCode HaitaiMotorController::getDeviceInfo(
    const std::string& motorId, DeviceInfo& info) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    uint8_t address;
    
    try {
        address = getAddress(id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    std::vector<uint8_t> responseData;
    ErrorCode result = sendAndReceive(0x0A, address, {}, responseData, id, 0x14);
    
    if (result != ErrorCode::SUCCESS) {
        return result;
    }
    
    if (responseData.size() < 20) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "设备信息响应数据长度不足");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    // 解析设备信息
    info.model = static_cast<uint16_t>(responseData[0] | (responseData[1] << 8));
    info.hwVersion = responseData[2];
    info.hwConfig = responseData[3];
    info.swVersion = static_cast<uint16_t>(responseData[4] | (responseData[5] << 8));
    
    info.mcuId.assign(responseData.begin() + 6, responseData.begin() + 18);
    info.rs485Version = responseData[18];
    info.canVersion = responseData[19];
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::readSystemData(
    const std::string& motorId, MotorPosition& position, MotorStatus& status) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    uint8_t address;
    
    try {
        address = getAddress(id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    std::vector<uint8_t> responseData;
    ErrorCode result = sendAndReceive(0x0B, address, {}, responseData, id, 0x0D);
    
    if (result != ErrorCode::SUCCESS) {
        return result;
    }
    
    if (responseData.size() < 13) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "系统数据响应长度不足");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    // 解析位置数据
    uint16_t singleTurnRaw = static_cast<uint16_t>(responseData[0] | (responseData[1] << 8));
    position.singleTurnAngle = singleTurnRaw * (360.0f / 16384.0f);
    
    int32_t multiTurnRaw = responseData[2] | (responseData[3] << 8) | 
                          (responseData[4] << 16) | (responseData[5] << 24);
    position.multiTurnAngle = multiTurnRaw * (360.0f / 16384.0f);
    
    int16_t speedRaw = static_cast<int16_t>(responseData[6] | (responseData[7] << 8));
    position.speed = speedRaw * 0.1f;
    
    // 解析状态数据
    status.voltage = responseData[8] * 0.2f;
    status.current = responseData[9] * 0.03f;
    status.temperature = responseData[10] * 0.4f;
    status.faultCode = responseData[11];
    status.runState = responseData[12];
    
    // 更新内部状态
    {
        std::lock_guard<std::mutex> lock(motorsMutex_);
        auto it = motors_.find(id);
        if (it != motors_.end()) {
            it->second.position = position;
            it->second.status = status;
            it->second.lastUpdateTime = std::chrono::steady_clock::now();
        }
    }
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::readSystemParams(
    const std::string& motorId, SystemParameters& params) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    uint8_t address;
    
    try {
        address = getAddress(id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    std::vector<uint8_t> responseData;
    ErrorCode result = sendAndReceive(0x0C, address, {}, responseData, id, 0x1A);
    
    if (result != ErrorCode::SUCCESS) {
        return result;
    }
    
    if (responseData.size() < 26) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "系统参数响应长度不足");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    // 解析系统参数
    params = parseSystemParameters(responseData);
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::writeSystemParamsTemp(
    const std::string& motorId, const SystemParameters& params) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    uint8_t address;
    
    try {
        address = getAddress(id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    // 打包参数
    std::vector<uint8_t> data = packSystemParameters(params);
    
    std::vector<uint8_t> responseData;
    ErrorCode result = sendAndReceive(0x0D, address, data, responseData, id, 0x1A);
    
    if (result != ErrorCode::SUCCESS) {
        return result;
    }
    
    // 验证响应
    if (responseData.size() < 26) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "系统参数响应长度不足");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::readMotorStatus(
    const std::string& motorId, MotorStatus& status) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    uint8_t address;
    
    try {
        address = getAddress(id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    std::vector<uint8_t> responseData;
    ErrorCode result = sendAndReceive(0x40, address, {}, responseData, id, 0x05);
    
    if (result != ErrorCode::SUCCESS) {
        return result;
    }
    
    if (responseData.size() < 5) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "电机状态响应长度不足");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    // 解析状态
    status.voltage = responseData[0] * 0.2f;
    status.current = responseData[1] * 0.03f;
    status.temperature = responseData[2] * 0.4f;
    status.faultCode = responseData[3];
    status.runState = responseData[4];
    
    // 更新内部状态
    {
        std::lock_guard<std::mutex> lock(motorsMutex_);
        auto it = motors_.find(id);
        if (it != motors_.end()) {
            it->second.status = status;
            it->second.lastUpdateTime = std::chrono::steady_clock::now();
        }
    }
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::clearFaults(
    const std::string& motorId) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    uint8_t address;
    
    try {
        address = getAddress(id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    std::vector<uint8_t> responseData;
    ErrorCode result = sendAndReceive(0x41, address, {}, responseData, id, 0x05);
    
    if (result != ErrorCode::SUCCESS) {
        return result;
    }
    
    if (responseData.size() < 5) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "清除故障响应长度不足");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    // 验证故障是否真正被清除
    uint8_t faultCode = responseData[3];
    if (faultCode != 0) {
        std::stringstream ss;
        ss << "故障未完全清除，剩余故障码: 0x" << std::hex << static_cast<int>(faultCode);
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, ss.str());
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    // 更新内部状态
    MotorStatus status;
    status.voltage = responseData[0] * 0.2f;
    status.current = responseData[1] * 0.03f;
    status.temperature = responseData[2] * 0.4f;
    status.faultCode = faultCode;
    status.runState = responseData[4];
    
    {
        std::lock_guard<std::mutex> lock(motorsMutex_);
        auto it = motors_.find(id);
        if (it != motors_.end()) {
            it->second.status = status;
            it->second.lastUpdateTime = std::chrono::steady_clock::now();
        }
    }
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::setCurrentPositionAsOrigin(
    const std::string& motorId) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    uint8_t address;
    
    try {
        address = getAddress(id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    std::vector<uint8_t> responseData;
    ErrorCode result = sendAndReceive(0x21, address, {}, responseData, id, 0x03);
    
    if (result != ErrorCode::SUCCESS) {
        return result;
    }
    
    if (responseData.size() < 3) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "设置原点响应长度不足");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    // 验证设置是否成功
    uint8_t success = responseData[2];
    if (success != 0x01) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "设置原点失败");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::speedClosedLoop(
    const std::string& motorId, float targetSpeed, MotorPosition& position) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    uint8_t address;
    
    try {
        address = getAddress(id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    // 验证速度限制
    if (!validateMotionLimits(id, 0, targetSpeed)) {
        return ErrorCode::MOTION_LIMIT_ERROR;
    }
    
    // 转换速度为电机内部格式
    int16_t speedRaw = static_cast<int16_t>(targetSpeed * 10);
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(speedRaw & 0xFF),
        static_cast<uint8_t>((speedRaw >> 8) & 0xFF)
    };
    
    std::vector<uint8_t> responseData;
    ErrorCode result = sendAndReceive(0x54, address, data, responseData, id, 0x08);
    
    if (result != ErrorCode::SUCCESS) {
        return result;
    }
    
    if (responseData.size() < 8) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "速度控制响应长度不足");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    // 解析位置数据
    uint16_t singleTurnRaw = static_cast<uint16_t>(responseData[0] | (responseData[1] << 8));
    position.singleTurnAngle = singleTurnRaw * (360.0f / 16384.0f);
    
    int32_t multiTurnRaw = responseData[2] | (responseData[3] << 8) | 
                          (responseData[4] << 16) | (responseData[5] << 24);
    position.multiTurnAngle = multiTurnRaw * (360.0f / 16384.0f);
    
    int16_t speedRawResponse = static_cast<int16_t>(responseData[6] | (responseData[7] << 8));
    position.speed = speedRawResponse * 0.1f;
    
    // 更新内部状态
    {
        std::lock_guard<std::mutex> lock(motorsMutex_);
        auto it = motors_.find(id);
        if (it != motors_.end()) {
            it->second.position = position;
            it->second.lastUpdateTime = std::chrono::steady_clock::now();
        }
    }
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::positionClosedLoop(
    const std::string& motorId, float targetPosition, float maxSpeed, MotorPosition& position) {
    
    std::string id = motorId.empty() ? defaultMotorId_ : motorId;
    uint8_t address;
    
    try {
        address = getAddress(id);
    } catch (const std::exception& e) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::PARAMETER_ERROR, e.what());
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    // 验证位置和速度限制
    if (!validateMotionLimits(id, targetPosition, maxSpeed)) {
        return ErrorCode::MOTION_LIMIT_ERROR;
    }
    
    // 转换位置为电机内部格式 (转角*16384/360)
    int32_t positionRaw = static_cast<int32_t>(targetPosition * 16384.0f / 360.0f);
    
    // 转换速度为电机内部格式
    uint16_t speedRaw = static_cast<uint16_t>(std::abs(maxSpeed) * 10);
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(positionRaw & 0xFF),
        static_cast<uint8_t>((positionRaw >> 8) & 0xFF),
        static_cast<uint8_t>((positionRaw >> 16) & 0xFF),
        static_cast<uint8_t>((positionRaw >> 24) & 0xFF),
        static_cast<uint8_t>(speedRaw & 0xFF),
        static_cast<uint8_t>((speedRaw >> 8) & 0xFF)
    };
    
    std::vector<uint8_t> responseData;
    ErrorCode result = sendAndReceive(0x53, address, data, responseData, id, 0x08);
    
    if (result != ErrorCode::SUCCESS) {
        return result;
    }
    
    if (responseData.size() < 8) {
        if (errorCallback_) {
            errorCallback_(id, ErrorCode::RESPONSE_ERROR, "位置控制响应长度不足");
        }
        return ErrorCode::RESPONSE_ERROR;
    }
    
    // 解析位置数据
    uint16_t singleTurnRaw = static_cast<uint16_t>(responseData[0] | (responseData[1] << 8));
    position.singleTurnAngle = singleTurnRaw * (360.0f / 16384.0f);
    
    int32_t multiTurnRaw = responseData[2] | (responseData[3] << 8) | 
                          (responseData[4] << 16) | (responseData[5] << 24);
    position.multiTurnAngle = multiTurnRaw * (360.0f / 16384.0f);
    
    int16_t speedRawResponse = static_cast<int16_t>(responseData[6] | (responseData[7] << 8));
    position.speed = speedRawResponse * 0.1f;
    
    // 更新内部状态
    {
        std::lock_guard<std::mutex> lock(motorsMutex_);
        auto it = motors_.find(id);
        if (it != motors_.end()) {
            it->second.position = position;
            it->second.lastUpdateTime = std::chrono::steady_clock::now();
        }
    }
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::synchronizedMove(
    const std::map<std::string, float>& motorPositions, float maxSpeed) {
    
    if (motorPositions.empty()) {
        if (errorCallback_) {
            errorCallback_("", ErrorCode::PARAMETER_ERROR, "电机位置映射为空");
        }
        return ErrorCode::PARAMETER_ERROR;
    }
    
    // 验证所有电机的位置限制
    for (const auto& pair : motorPositions) {
        if (!validateMotionLimits(pair.first, pair.second, maxSpeed)) {
            return ErrorCode::MOTION_LIMIT_ERROR;
        }
    }
    
    // 发送所有电机的位置命令，使用多线程并行发送以减少总延迟
    std::vector<std::thread> threads;
    std::map<std::string, ErrorCode> results;
    std::mutex resultsMutex;
    
    for (const auto& pair : motorPositions) {
        threads.emplace_back([this, &pair, maxSpeed, &results, &resultsMutex]() {
            MotorPosition position;
            ErrorCode result = positionClosedLoop(pair.first, pair.second, maxSpeed, position);
            
            std::lock_guard<std::mutex> lock(resultsMutex);
            results[pair.first] = result;
        });
    }
    
    // 等待所有命令完成
    for (auto& thread : threads) {
        thread.join();
    }
    
    // 检查结果
    for (const auto& pair : results) {
        if (pair.second != ErrorCode::SUCCESS) {
            if (errorCallback_) {
                errorCallback_(pair.first, pair.second, 
                             "同步移动失败: " + getErrorMessage(pair.second));
            }
            return pair.second;
        }
    }
    
    return ErrorCode::SUCCESS;
}

HaitaiMotorController::ErrorCode HaitaiMotorController::shutdownAllMotors() {
    ErrorCode finalResult = ErrorCode::SUCCESS;
    
    std::vector<std::string> motorIds;
    {
        std::lock_guard<std::mutex> lock(motorsMutex_);
        for (const auto& pair : motors_) {
            motorIds.push_back(pair.first);
        }
    }
    
    for (const std::string& id : motorIds) {
        uint8_t address;
        
        try {
            address = getAddress(id);
        } catch (const std::exception&) {
            continue;  // 跳过无效电机
        }
        
        std::vector<uint8_t> responseData;
        ErrorCode result = sendAndReceive(0x50, address, {}, responseData, id, 0x08);
        
        if (result != ErrorCode::SUCCESS && finalResult == ErrorCode::SUCCESS) {
            finalResult = result;
        }
    }
    
    return finalResult;
}
