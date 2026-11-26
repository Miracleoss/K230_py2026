#ifndef USER_COMMON_ATTITUDE_STATE_H
#define USER_COMMON_ATTITUDE_STATE_H

#include <stdint.h>

/**
 * @brief 飞行器姿态状态结构体
 *
 * 描述飞行器当前的姿态角（欧拉角）与姿态角速度，供飞控的姿态估计与闭环控制使用。
 * 角度单位均为弧度，角速度单位为弧度每秒。
 */
struct AttitudeState {
    float roll_rad;    ///< 滚转角（rad）
    float pitch_rad;   ///< 俯仰角（rad）
    float yaw_rad;     ///< 偏航角（rad）

    float roll_rate_radps;   ///< 滚转角速度（rad/s）
    float pitch_rate_radps;  ///< 俯仰角速度（rad/s）
    float yaw_rate_radps;    ///< 偏航角速度（rad/s）

    uint32_t timestamp_ms;   ///< 时间戳（ms）

    bool valid;              ///< 数据有效性标志

    AttitudeState()
        : roll_rad(0.0f),
          pitch_rad(0.0f),
          yaw_rad(0.0f),
          roll_rate_radps(0.0f),
          pitch_rate_radps(0.0f),
          yaw_rate_radps(0.0f),
          timestamp_ms(0U),
          valid(false)
    {
    }
};

#endif /* USER_COMMON_ATTITUDE_STATE_H */
