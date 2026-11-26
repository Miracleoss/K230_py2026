#ifndef USER_COMMON_SYSTEM_STATUS_H
#define USER_COMMON_SYSTEM_STATUS_H

#include <stdint.h>

#include "Common/common_types.h"

/**
 * @brief 系统状态
 *
 * 汇总飞控当前运行状态，供上位机查询与状态显示。
 */
struct SystemStatus {
    FlightMode mode;  ///< 当前飞行模式

    bool imu_ready;     ///< IMU 是否就绪
    bool heater_ready;  ///< 加热器是否就绪
    bool host_alive;    ///< 上位机连接是否存活
    bool servo_ready;   ///< 舵机是否就绪

    FaultFlags fault_flags;  ///< 故障标志位（按位组合）
    ResultCode last_error;   ///< 最近一次错误码

    uint32_t uptime_ms;      ///< 系统运行时间（ms）
    uint32_t overrun_count;  ///< 任务超时（调度超时）计数

    SystemStatus()
        : mode(FlightMode::Init),
          imu_ready(false),
          heater_ready(false),
          host_alive(false),
          servo_ready(false),
          fault_flags(0U),
          last_error(ResultCode::Ok),
          uptime_ms(0U),
          overrun_count(0U)
    {
    }
};

#endif /* USER_COMMON_SYSTEM_STATUS_H */
