#ifndef USER_COMMON_SURFACE_COMMAND_H
#define USER_COMMON_SURFACE_COMMAND_H

#include <stdint.h>

#include "Common/common_types.h"

/**
 * @brief 根据舵面标识获取其数组下标
 * @param surface_id     舵面标识
 * @param[out] surface_index 输出：对应的数组下标（失败时为 0）
 * @return true 表示转换成功，false 表示 surface_id 非法
 */
inline bool TryGetSurfaceIndex(SurfaceId surface_id, uint32_t& surface_index)
{
    switch (surface_id) {
    case SurfaceId::Servo1RightUpper:
        surface_index = 0U;
        return true;
    case SurfaceId::Servo2LeftUpper:
        surface_index = 1U;
        return true;
    case SurfaceId::Servo3LeftLower:
        surface_index = 2U;
        return true;
    case SurfaceId::Servo4RightLower:
        surface_index = 3U;
        return true;
    default:
        surface_index = 0U;
        return false;
    }
}

/**
 * @brief 根据数组下标获取对应的舵面标识
 * @param surface_index 数组下标（0~3）
 * @param[out] surface_id 输出：对应的舵面标识（失败时为 Servo1RightUpper）
 * @return true 表示转换成功，false 表示 surface_index 越界
 */
inline bool TryGetSurfaceId(uint32_t surface_index, SurfaceId& surface_id)
{
    switch (surface_index) {
    case 0U:
        surface_id = SurfaceId::Servo1RightUpper;
        return true;
    case 1U:
        surface_id = SurfaceId::Servo2LeftUpper;
        return true;
    case 2U:
        surface_id = SurfaceId::Servo3LeftLower;
        return true;
    case 3U:
        surface_id = SurfaceId::Servo4RightLower;
        return true;
    default:
        surface_id = SurfaceId::Servo1RightUpper;
        return false;
    }
}

/**
 * @brief 舵面指令
 *
 * 数组下标顺序为：Servo1 右上、Servo2 左上、Servo3 左下、Servo4 右下。
 * delta 表示各舵面产生正切向气动力的归一化比例，范围为 [-1, +1]。
 * 它不是舵机角度、舵机轴角度或 PWM 偏移量。
 */
struct SurfaceCommand {
    float delta1;  ///< Servo1（右上）的归一化正切向气动力指令
    float delta2;  ///< Servo2（左上）的归一化正切向气动力指令
    float delta3;  ///< Servo3（左下）的归一化正切向气动力指令
    float delta4;  ///< Servo4（右下）的归一化正切向气动力指令

    uint32_t timestamp_ms;  ///< 时间戳（ms）

    bool valid;             ///< 指令有效性标志

    SurfaceCommand()
        : delta1(0.0f),
          delta2(0.0f),
          delta3(0.0f),
          delta4(0.0f),
          timestamp_ms(0U),
          valid(false)
    {
    }
};

#endif /* USER_COMMON_SURFACE_COMMAND_H */
