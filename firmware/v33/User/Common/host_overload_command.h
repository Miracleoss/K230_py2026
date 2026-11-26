#ifndef USER_COMMON_HOST_OVERLOAD_COMMAND_H
#define USER_COMMON_HOST_OVERLOAD_COMMAND_H

#include <stdint.h>

/**
 * @brief Host-provided body-axis overload targets in physical g units.
 *
 * These values are deliberately separate from BodyControlCommand, whose
 * roll/pitch/yaw fields are normalized to [-1, +1].
 */
struct HostOverloadCommand {
    uint8_t is_launched;              ///< 第 1 字节（帧偏移 2）：是否已经发射（0: 未发射, 非0: 已发射）
    uint8_t is_base_green_detected;   ///< 第 2 字节（帧偏移 3）：是否识别到基地绿灯（0: 未识别, 非0: 已识别）
    float yaw_overload_g;
    float pitch_overload_g;
    uint8_t reserved0;
    uint8_t reserved1;
    bool valid;

    HostOverloadCommand()
        : is_launched(0U),
          is_base_green_detected(0U),
          yaw_overload_g(0.0f),
          pitch_overload_g(0.0f),
          reserved0(0U),
          reserved1(0U),
          valid(false)
    {
    }

    bool IsLaunched() const { return is_launched != 0U; }
    bool IsBaseGreenDetected() const { return is_base_green_detected != 0U; }
    bool IsGreenLightDetected() const { return is_base_green_detected != 0U; }
};

#endif /* USER_COMMON_HOST_OVERLOAD_COMMAND_H */
