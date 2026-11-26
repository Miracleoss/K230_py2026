#include "Config/servo_config.h"

/**
 * @brief 四路 X 型尾翼舵机配置
 *
 * 观察方向为从镖尾看向镖头。delta>0 表示舵面产生正切向气动力：
 * Servo1 右下、Servo2 右上、Servo3 左上、Servo4 左下。
 * 实测四路的 delta 正方向都要求 PWM 减小，因此 reversed 均为 true；
 * 对应后缘方向依次为上、下、下、上。每路有效范围均为 neutral±500us。
 */
const ServoConfig kServoConfigs[kSurfaceCount] = {
    {
        SurfaceId::Servo1RightUpper,
        PwmChannelId::Tim1Ch1,
        333U,
        2U,
        true,
        1650U,
        0,
        1150U,
        2150U,
        0.0f,
        0.0f,
        0.0f,
        true ///< delta+：后缘向上，PWM 减小
    },
    {
        SurfaceId::Servo2LeftUpper,
        PwmChannelId::Tim1Ch2,
        333U,
        2U,
        true,
        1450U,
        0,
        950U,
        1950U,
        0.0f,
        0.0f,
        0.0f,
        true ///< delta+：后缘向下，PWM 减小
    },
    {
        SurfaceId::Servo3LeftLower,
        PwmChannelId::Tim1Ch3,
        333U,
        2U,
        true,
        1450U,
        0,
        950U,
        1950U,
        0.0f,
        0.0f,
        0.0f,
        true ///< delta+：后缘向下，PWM 减小
    },
    {
        SurfaceId::Servo4RightLower,
        PwmChannelId::Tim1Ch4,
        333U,
        2U,
        true,
        1510U,
        0,
        1010U,
        2010U,
        0.0f,
        0.0f,
        0.0f,
        true ///< delta+：后缘向上，PWM 减小
    }
};
