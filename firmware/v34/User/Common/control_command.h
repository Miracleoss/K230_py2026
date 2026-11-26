#ifndef USER_COMMON_CONTROL_COMMAND_H
#define USER_COMMON_CONTROL_COMMAND_H

#include <stdint.h>

/**
 * @brief 机体控制指令
 *
 * 由上层控制算法输出的归一化机体三轴控制请求，范围为 [-1,+1]，
 * 后续交由 X 尾翼混控器（XTailMixer）换算为各舵面的 delta。
 */
struct BodyControlCommand {
    float roll;   ///< 滚转控制需求（绕机体 +X 前向轴，顺时针为正）
    float pitch;  ///< 俯仰控制需求（绕机体 +Z 右向轴，抬头向上为正）
    float yaw;    ///< 偏航控制需求（绕机体 +Y 上向轴，机头向左为正）

    uint32_t timestamp_ms;  ///< 时间戳（ms）

    bool valid;             ///< 指令有效性标志

    BodyControlCommand()
        : roll(0.0f),
          pitch(0.0f),
          yaw(0.0f),
          timestamp_ms(0U),
          valid(false)
    {
    }
};

#endif /* USER_COMMON_CONTROL_COMMAND_H */
