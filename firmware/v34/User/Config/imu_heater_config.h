#ifndef USER_CONFIG_IMU_HEATER_CONFIG_H
#define USER_CONFIG_IMU_HEATER_CONFIG_H

#include <stdint.h>

/** @brief IMU 加热器温度闭环的更新周期，单位 ms（20 Hz）。 */
static const uint32_t kImuHeaterUpdatePeriodMs = 50U;

/**
 * @brief IMU 加热器 PI 温度控制器配置。
 *
 * 控制器把“目标温度 - 当前温度”的误差换算成加热 PWM 占空比；加热器只能加热
 * 不能制冷，因此积分下限固定为 0，超调后靠自然散热回落到目标温度。
 */
struct ImuHeaterConfig {
    /** @brief 目标温度，单位摄氏度。 */
    float target_temperature_deg_c;
    /** @brief 超温安全关断阈值，单位摄氏度；达到即强制占空比 0。 */
    float max_temperature_deg_c;
    /** @brief 比例增益，占空比/°C。 */
    float kp;
    /** @brief 积分增益，占空比/(°C·s)。 */
    float ki;
    /** @brief 积分下限，单位 °C·s；加热器不允许负积分，取 0。 */
    float integral_min;
    /** @brief 积分上限，单位 °C·s，用于抗饱和。 */
    float integral_max;
    /** @brief 占空比输出下限，范围 [0, 1]。 */
    float duty_min;
    /** @brief 占空比输出上限，范围 [0, 1]。 */
    float duty_max;
    /** @brief 接受的最短控制周期，单位 s。 */
    float dt_min_s;
    /** @brief 接受的最长控制周期，单位 s。 */
    float dt_max_s;
};

/**
 * @brief 创建默认 IMU 加热器温度控制配置。
 * @return 目标温度 45°C 的默认配置；kp/ki 为保守占位值，需在硬件上整定。
 */
ImuHeaterConfig MakeDefaultImuHeaterConfig();

#endif /* USER_CONFIG_IMU_HEATER_CONFIG_H */
