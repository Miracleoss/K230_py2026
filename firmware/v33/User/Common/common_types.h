#ifndef USER_COMMON_COMMON_TYPES_H
#define USER_COMMON_COMMON_TYPES_H

#include <stdint.h>

/**
 * @brief 通用结果码
 *
 * 所有驱动与模块接口统一使用该枚举作为返回值，便于上层统一处理错误。
 */
enum class ResultCode {
    Ok,            ///< 操作成功
    NotReady,      ///< 设备尚未就绪（忙）
    Timeout,       ///< 操作超时
    InvalidData,   ///< 输入数据非法
    HardwareError, ///< 硬件错误
    ConfigError    ///< 配置错误
};

/**
 * @brief 飞行模式
 */
enum class FlightMode {
    Init,     ///< 初始化阶段
    Ready,    ///< 就绪（已标定/自检通过，等待起飞）
    Active,   ///< 激活（正常飞行控制中）
    Failsafe  ///< 故障保护（失联/异常进入安全状态）
};

/**
 * @brief 舵面（控制面）标识
 *
 * 对应机体上的四个舵面，编号与坐标约定一致：
 *   Servo1 右上、Servo2 左上、Servo3 左下、Servo4 右下。
 */
enum class SurfaceId {
    Servo1RightUpper, ///< 舵机 1：右上
    Servo2LeftUpper,  ///< 舵机 2：左上
    Servo3LeftLower,  ///< 舵机 3：左下
    Servo4RightLower  ///< 舵机 4：右下
};

/// 舵面（舵机）总数
static const uint32_t kSurfaceCount = 4U;

/**
 * @brief 故障码（位标志）
 *
 * 每个故障对应 FaultFlags 中的一位，便于按位组合与查询。
 */
enum class FaultCode : uint32_t {
    None             = 0U,          ///< 无故障
    ImuInit          = 1UL << 0,    ///< IMU 初始化失败
    ImuStale         = 1UL << 1,    ///< IMU 数据过期（无新数据）
    HostTimeout      = 1UL << 2,    ///< 上位机通信超时
    InvalidHostData  = 1UL << 3,    ///< 上位机数据非法
    ControllerInvalid = 1UL << 4,   ///< 控制器（算法）失效
    TaskOverrun      = 1UL << 5     ///< 任务运行超时（调度超时）
};

/// 故障标志位类型（按位组合多个 FaultCode）
typedef uint32_t FaultFlags;

/**
 * @brief 将单个故障码转换为故障标志位
 * @param fault_code 故障码
 * @return 对应的位掩码
 */
inline FaultFlags FaultMask(FaultCode fault_code)
{
    return static_cast<FaultFlags>(fault_code);
}

/**
 * @brief 判断故障标志中是否包含指定故障
 * @param fault_flags 当前故障标志位
 * @param fault_code  待查询的故障码
 * @return true 表示存在该故障，false 表示不存在
 */
inline bool HasFault(FaultFlags fault_flags, FaultCode fault_code)
{
    return (fault_flags & FaultMask(fault_code)) != 0U;
}

#endif /* USER_COMMON_COMMON_TYPES_H */
