#ifndef USER_DEVICE_DEV_SERVO_H
#define USER_DEVICE_DEV_SERVO_H

#include "Common/common_types.h"
#include "Config/servo_config.h"
#include "Driver/drv_pwm_channel.h"

class Class_Servo {
public:
    Class_Servo();

    ResultCode Bind(Class_PwmChannel* pwm, const ServoConfig* config);
    bool IsBound() const;

    /**
     * @brief 输出归一化舵面正切向气动力指令
     * @param delta 正切向气动力比例；超出 [-1,+1] 时饱和
     */
    ResultCode SetDelta(float delta);

private:
    Class_PwmChannel* pwm_;
    const ServoConfig* config_;
};

#endif /* USER_DEVICE_DEV_SERVO_H */
