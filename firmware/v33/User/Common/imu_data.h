#ifndef USER_COMMON_IMU_DATA_H
#define USER_COMMON_IMU_DATA_H

#include <stdint.h>

/**
 * @brief 统一后的 IMU 数据。
 *
 * 这里的 X/Y/Z 始终表示整机机体坐标系，输入边界之外不得再按 BMI088
 * 传感器坐标系重新解释或换轴。当前数据只完成标称灵敏度换算和坐标变换，
 * 不表示已经完成零偏、比例因子、温漂等传感器标定。
 */
struct ImuData {
    float specific_force_x_mps2;  ///< 机体 X 轴比力（m/s^2）。
    float specific_force_y_mps2;  ///< 机体 Y 轴比力（m/s^2）。
    float specific_force_z_mps2;  ///< 机体 Z 轴比力（m/s^2）。
    float gyro_x_radps;  ///< 机体 X 轴角速度（rad/s）。
    float gyro_y_radps;  ///< 机体 Y 轴角速度（rad/s）。
    float gyro_z_radps;  ///< 机体 Z 轴角速度（rad/s）。
    float temperature_deg_c;  ///< 传感器温度（摄氏度）。
    uint32_t sample_timestamp_ms;  ///< 采样毫秒时间戳（ms）。
    uint64_t sample_timestamp_us;  ///< 采样微秒时间戳（us，硬件 1 us 分辨率）。
    uint32_t data_ready_seq;       ///< 成功采样序号（递增计数）。
    bool valid;  ///< 该完整样本是否有效。

    void Setspecific_force_x_mps2(float value) { specific_force_x_mps2 = value; }
    void Setspecific_force_y_mps2(float value) { specific_force_y_mps2 = value; }
    void Setspecific_force_z_mps2(float value) { specific_force_z_mps2 = value; }

    ImuData()
        : specific_force_x_mps2(0.0f),
          specific_force_y_mps2(0.0f),
          specific_force_z_mps2(0.0f),
          gyro_x_radps(0.0f),
          gyro_y_radps(0.0f),
          gyro_z_radps(0.0f),
          temperature_deg_c(0.0f),
          sample_timestamp_ms(0U),
          sample_timestamp_us(0ULL),
          data_ready_seq(0U),
          valid(false)
    {
    }
};

#endif /* USER_COMMON_IMU_DATA_H */
