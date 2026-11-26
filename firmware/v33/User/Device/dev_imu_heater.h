#ifndef USER_DEVICE_DEV_IMU_HEATER_H
#define USER_DEVICE_DEV_IMU_HEATER_H

#include "Common/common_types.h"
#include "Config/imu_heater_config.h"
#include "Driver/drv_pwm_channel.h"

class Class_ImuHeater {
public:
    Class_ImuHeater();

    ResultCode Bind(Class_PwmChannel* pwm);
    bool IsBound() const;

    /// 载入温度控制配置并复位积分状态
    ResultCode Configure(const ImuHeaterConfig& config);

    /// 启动 PWM 输出（初始占空比保持为 0）
    ResultCode Start();

    /// 停止 PWM 输出
    ResultCode Stop();

    /**
     * @brief 依据当前温度执行一次 PI 温度闭环，更新加热占空比。
     * @param temperature_deg_c 当前 IMU 温度，单位摄氏度。
     * @param dt_s 距上次更新的时间，单位 s；会被限幅到配置的 [dt_min_s, dt_max_s]。
     * @return 未绑定时返回 NotReady；输入非有限时返回 InvalidData；否则 Ok。
     */
    ResultCode Update(float temperature_deg_c, float dt_s);

private:
    Class_PwmChannel* pwm_;
    ImuHeaterConfig config_;
    float integral_;
};

#endif /* USER_DEVICE_DEV_IMU_HEATER_H */
