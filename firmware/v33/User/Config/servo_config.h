#ifndef USER_CONFIG_SERVO_CONFIG_H
#define USER_CONFIG_SERVO_CONFIG_H

#include <stdint.h>

#include "Common/common_types.h"
#include "Config/board_hardware_config.h"

/**
 * @brief 舵机配置
 *
 * 描述单个舵机的逻辑-物理映射、脉冲参数、飞行偏转范围及换算关系。
 */
struct ServoConfig {
    SurfaceId surface_id;          ///< 对应的舵面标识
    PwmChannelId pwm_channel_id;   ///< 绑定的 PWM 通道标识

    uint16_t refresh_rate_hz;      ///< PWM 刷新频率（Hz）
    uint16_t deadband_us;          ///< 死区（us），用于抑制微小抖动

    bool reversed;                 ///< true：delta 增大时脉宽减小

    uint16_t neutral_pulse_us;     ///< delta=0 对应的中立位脉宽（us）
    int16_t trim_pulse_us;         ///< 微调脉宽（us，可为负）

    uint16_t physical_min_pulse_us; ///< delta=+1 对应的最小脉宽（us）
    uint16_t physical_max_pulse_us; ///< delta=-1 对应的最大脉宽（us）

    float flight_min_deflection_rad; ///< 保留字段，本阶段不参与 delta 映射
    float flight_max_deflection_rad; ///< 保留字段，本阶段不参与 delta 映射
    float rad_to_pulse_width_us;     ///< 保留字段，本阶段不参与 delta 映射

    bool calibrated;               ///< 是否已完成标定
};

/// 全部舵机配置表（数量为 kSurfaceCount）
extern const ServoConfig kServoConfigs[kSurfaceCount];

#endif /* USER_CONFIG_SERVO_CONFIG_H */
