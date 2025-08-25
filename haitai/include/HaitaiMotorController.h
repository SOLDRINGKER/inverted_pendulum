// HaitaiMotorController.h
#ifndef HAITAI_MOTOR_CONTROLLER_H
#define HAITAI_MOTOR_CONTROLLER_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <fstream>

class HaitaiMotorController {
public:
    // 常量定义 - 使用static const替代static constexpr以避免链接错误
    static const uint8_t HEADER_HOST;
    static const uint8_t HEADER_SLAVE;
    
    // 错误码枚举
    enum class ErrorCode {
        SUCCESS = 0,
        SERIAL_ERROR,
        TIMEOUT_ERROR,
        CRC_ERROR,
        RESPONSE_ERROR,
        PARAMETER_ERROR,
        MOTION_LIMIT_ERROR,
        COMMUNICATION_ERROR,
        UNKNOWN_ERROR
    };
    
    // 数据结构定义
    struct MotorStatus {
        float voltage;          // 电压 (V)
        float current;          // 电流 (A)
        float temperature;      // 温度 (°C)
        uint8_t faultCode;      // 故障码
        uint8_t runState;       // 运行状态
    };
    
    struct MotorPosition {
        float singleTurnAngle;  // 单圈角度 (°)
        float multiTurnAngle;   // 多圈角度 (°)
        float speed;            // 速度 (RPM)
    };
    
    struct SystemParameters {
        uint8_t deviceAddress;       // 设备地址
        float currentThreshold;      // 电流阈值 (A)
        float voltageThreshold;      // 电压阈值 (V)
        uint8_t baudrates;           // 波特率
        float positionKp;            // 位置环P参数
        float positionTargetSpeed;   // 位置环目标速度
        float speedKp;               // 速度环P参数
        float speedKi;               // 速度环I参数
        float reserved;              // 保留参数
        float speedFilter;           // 速度滤波系数
        uint8_t powerPercent;        // 功率百分比
    };
    
    struct DeviceInfo {
        uint16_t model;             // 型号
        uint8_t hwVersion;          // 硬件版本
        uint8_t hwConfig;           // 硬件配置
        uint16_t swVersion;         // 软件版本
        std::vector<uint8_t> mcuId; // MCU ID
        uint8_t rs485Version;       // RS485版本
        uint8_t canVersion;         // CAN版本
    };
    
    struct MotorLimits {
        float minAngle = -1000000.0f;  // 最小角度限制 (°)
        float maxAngle = 1000000.0f;   // 最大角度限制 (°)
        float maxSpeed = 3000.0f;      // 最大速度限制 (RPM)
    };

    // 参数配置结构体
    struct ControlParameters {
        // PD控制参数
        struct PDParams {
            float kp = 0.7f;
            float kd = 0.05f;
        };
        
        // 速度环PID参数
        struct SpeedPIDParams {
            float kp = 0.8f;
            float ki = 0.2f;
            float filter = 0.9f;
        };
        
        // 轨迹参数
        struct TrajectoryParams {
            float amplitude = 100.0f;   // 振幅(度)
            float frequency = 1.0f;     // 频率(Hz)
            float phase = 0.0f;         // 相位(弧度)
        };
        
        // 安全参数
        struct SafetyParams {
            float maxPositionError = 25.0f;   // 最大位置误差(度)
            float errorTimeout = 2.0f;        // 误差超时时间(秒)
            float maxSpeed = 3000.0f;         // 最大速度(RPM)
        };
        
        std::map<std::string, PDParams> pdParams;
        std::map<std::string, TrajectoryParams> trajectoryParams;
        SpeedPIDParams speedPID;
        SafetyParams safety;
        
        // 控制参数
        float controlFrequency = 100.0f;  // 控制频率(Hz)
        float duration = 15.0f;           // 运行时间(秒)
    };

    // 回调函数类型
    using StatusCallback = std::function<void(const std::string&, const MotorStatus&)>;
    using ErrorCallback = std::function<void(const std::string&, ErrorCode, const std::string&)>;

    /**
     * @brief 构造函数
     * @param port 串口名 (Linux: "/dev/ttyUSB0")
     * @param baudrate 波特率，默认115200
     * @param motorAddresses 电机ID到地址的映射
     * @param defaultAddress 默认电机地址
     * @param timeout 通信超时时间(ms)
     * @param retries 通信重试次数
     */
    HaitaiMotorController(
        const std::string& port,
        int baudrate = 115200,
        const std::map<std::string, uint8_t>& motorAddresses = {},
        uint8_t defaultAddress = 0x02,
        unsigned int timeout = 50,
        unsigned int retries = 3
    );
    
    /**
     * @brief 析构函数，释放资源
     */
    ~HaitaiMotorController();
    
    /**
     * @brief 打开串口连接
     * @return 成功返回true，失败返回false
     */
    bool open();
    
    /**
     * @brief 关闭串口连接
     */
    void close();
    
    /**
     * @brief 检查串口是否已打开
     * @return 已打开返回true
     */
    bool isOpen() const;
    
    /**
     * @brief 添加电机
     * @param motorId 电机ID
     * @param address 电机地址
     * @return 成功返回true
     */
    bool addMotor(const std::string& motorId, uint8_t address);
    
    /**
     * @brief 设置电机运动限制
     * @param motorId 电机ID
     * @param limits 限制参数
     * @return 错误码
     */
    ErrorCode setMotorLimits(const std::string& motorId, const MotorLimits& limits);
    
    /**
     * @brief 获取设备信息
     * @param motorId 电机ID，默认为空使用默认电机
     * @param info 返回的设备信息
     * @return 错误码
     */
    ErrorCode getDeviceInfo(const std::string& motorId, DeviceInfo& info);
    ErrorCode getDeviceInfo(DeviceInfo& info) { return getDeviceInfo("", info); }
    
    /**
     * @brief 读取系统数据
     * @param motorId 电机ID，默认为空使用默认电机
     * @param position 返回的位置信息
     * @param status 返回的状态信息
     * @return 错误码
     */
    ErrorCode readSystemData(const std::string& motorId, MotorPosition& position, MotorStatus& status);
    ErrorCode readSystemData(MotorPosition& position, MotorStatus& status) { 
        return readSystemData("", position, status); 
    }
    
    /**
     * @brief 读取系统参数
     * @param motorId 电机ID
     * @param params 返回的参数
     * @return 错误码
     */
    ErrorCode readSystemParams(const std::string& motorId, SystemParameters& params);
    ErrorCode readSystemParams(SystemParameters& params) { return readSystemParams("", params); }
    
    /**
     * @brief 写入临时系统参数
     * @param motorId 电机ID
     * @param params 要写入的参数
     * @return 错误码
     */
    ErrorCode writeSystemParamsTemp(const std::string& motorId, const SystemParameters& params);
    ErrorCode writeSystemParamsTemp(const SystemParameters& params) { 
        return writeSystemParamsTemp("", params); 
    }
    
    /**
     * @brief 读取电机状态
     * @param motorId 电机ID
     * @param status 返回的状态
     * @return 错误码
     */
    ErrorCode readMotorStatus(const std::string& motorId, MotorStatus& status);
    ErrorCode readMotorStatus(MotorStatus& status) { return readMotorStatus("", status); }
    
    /**
     * @brief 清除故障并验证
     * @param motorId 电机ID
     * @return 错误码
     */
    ErrorCode clearFaults(const std::string& motorId);
    ErrorCode clearFaults() { return clearFaults(""); }
    
    /**
     * @brief 设置当前位置为原点
     * @param motorId 电机ID
     * @return 错误码
     */
    ErrorCode setCurrentPositionAsOrigin(const std::string& motorId);
    ErrorCode setCurrentPositionAsOrigin() { return setCurrentPositionAsOrigin(""); }
    
    /**
     * @brief 速度闭环控制
     * @param motorId 电机ID
     * @param targetSpeed 目标速度(RPM)
     * @param position 返回的位置信息
     * @return 错误码
     */
    ErrorCode speedClosedLoop(const std::string& motorId, float targetSpeed, MotorPosition& position);
    ErrorCode speedClosedLoop(float targetSpeed, MotorPosition& position) {
        return speedClosedLoop("", targetSpeed, position);
    }
    
    /**
     * @brief 位置闭环控制
     * @param motorId 电机ID
     * @param targetPosition 目标位置(°)
     * @param maxSpeed 最大速度(RPM)
     * @param position 返回的位置信息
     * @return 错误码
     */
    ErrorCode positionClosedLoop(const std::string& motorId, float targetPosition, 
                               float maxSpeed, MotorPosition& position);
    ErrorCode positionClosedLoop(float targetPosition, float maxSpeed, MotorPosition& position) {
        return positionClosedLoop("", targetPosition, maxSpeed, position);
    }
    
    /**
     * @brief 同步多电机运动
     * @param motorPositions 电机ID到目标位置的映射
     * @param maxSpeed 最大速度(RPM)
     * @return 错误码
     */
    ErrorCode synchronizedMove(const std::map<std::string, float>& motorPositions, float maxSpeed);
    
    /**
     * @brief 关闭所有电机
     * @return 错误码
     */
    ErrorCode shutdownAllMotors();
    
    /**
     * @brief 启动状态监控线程
     * @param interval 监控间隔(ms)
     * @param callback 状态回调函数
     * @return 成功返回true
     */
    bool startStatusMonitoring(unsigned int interval, StatusCallback callback);
    
    /**
     * @brief 停止状态监控线程
     */
    void stopStatusMonitoring();
    
    /**
     * @brief 设置错误回调
     * @param callback 错误回调函数
     */
    void setErrorCallback(ErrorCallback callback);
    
    /**
     * @brief 设置电机PD控制参数
     * @param motorId 电机ID
     * @param kp 位置比例系数
     * @param kd 位置微分系数
     * @return 是否成功
     */
    bool setPDParams(const std::string& motorId, float kp, float kd);
    
    /**
     * @brief 设置速度环PID参数
     * @param motorId 电机ID
     * @param params 速度环参数
     * @return 错误码
     */
    ErrorCode setSpeedPIDParams(const std::string& motorId, const ControlParameters::SpeedPIDParams& params);
    
    /**
     * @brief 加载参数配置
     * @param params 参数配置
     */
    void loadParameters(const ControlParameters& params);
    
    /**
     * @brief 获取当前参数配置
     * @return 当前参数配置
     */
    ControlParameters getCurrentParameters() const;
    
    /**
     * @brief 保存参数到文件
     * @param filename 文件名
     * @return 是否成功
     */
    bool saveParametersToFile(const std::string& filename) const;
    
    /**
     * @brief 从文件加载参数
     * @param filename 文件名
     * @return 是否成功
     */
    bool loadParametersFromFile(const std::string& filename);
    
    /**
     * @brief 获取错误信息字符串
     * @param code 错误码
     * @return 错误描述
     */
    static std::string getErrorMessage(ErrorCode code);

private:
    // 内部电机状态跟踪
    struct MotorState {
        uint8_t address;
        uint8_t packetSeq = 0;
        MotorPosition position;
        MotorStatus status;
        MotorLimits limits;
        std::chrono::time_point<std::chrono::steady_clock> lastUpdateTime;
    };
    
    // 控制参数成员变量
    ControlParameters controlParams_;
    
    // 私有成员变量
    std::string port_;
    int baudrate_;
    unsigned int timeout_;
    unsigned int retries_;
    std::map<std::string, MotorState> motors_;
    std::string defaultMotorId_;
    
    // 线程相关
    std::atomic<bool> monitoringActive_;
    std::unique_ptr<std::thread> monitoringThread_;
    StatusCallback statusCallback_;
    ErrorCallback errorCallback_;
    
    // 串口相关
    class SerialPort;
    std::unique_ptr<SerialPort> serial_;
    std::mutex serialMutex_;
    mutable std::mutex motorsMutex_;  // 修改为mutable以解决const方法中的锁问题
    
    // CRC查表
    static const uint16_t CRC16_TABLE[256];
    
    // 辅助方法
    uint8_t getAddress(const std::string& motorId);
    uint8_t nextSequence(const std::string& motorId);
    std::vector<uint8_t> buildPacket(uint8_t cmd, uint8_t address, 
                                    const std::vector<uint8_t>& data, 
                                    const std::string& motorId);
    uint16_t calculateCRC(const std::vector<uint8_t>& data);
    
    ErrorCode sendAndReceive(uint8_t cmd, uint8_t address, 
                           const std::vector<uint8_t>& sendData, 
                           std::vector<uint8_t>& receiveData,
                           const std::string& motorId,
                           unsigned int expectedResponseLen = 0);
    
    void monitoringFunction(unsigned int interval);
    bool validateMotionLimits(const std::string& motorId, float targetPosition, float targetSpeed);
    
    // 参数转换辅助函数
    SystemParameters parseSystemParameters(const std::vector<uint8_t>& data);
    std::vector<uint8_t> packSystemParameters(const SystemParameters& params);
    
    // 禁止拷贝构造和赋值
    HaitaiMotorController(const HaitaiMotorController&) = delete;
    HaitaiMotorController& operator=(const HaitaiMotorController&) = delete;
};

#endif // HAITAI_MOTOR_CONTROLLER_H
