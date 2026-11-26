#ifndef USER_CONFIG_CONTROL_CONFIG_H
#define USER_CONFIG_CONTROL_CONFIG_H

#include <stdint.h>

/** @brief 重力加速度换算常量，单位 m/s^2。 */
static const float kControlGravityMps2 = 9.81f;

/** @brief 过载指令允许下限，单位 g。 */
static const float kControlOverloadCommandMinG = -327.67f;
/** @brief 过载指令允许上限，单位 g。 */
static const float kControlOverloadCommandMaxG = 327.67f;
/** @brief PI 外环角速度指令下限，单位 rad/s。 */
static const float kControlRateCommandMinRadps = -100.0f;//原本是20
/** @brief PI 外环角速度指令上限，单位 rad/s。 */
static const float kControlRateCommandMaxRadps = 100.0f;//原本是20
/** @brief PI 外环积分累积下限，单位 g·s（过载倍数·s）。 */
static const float kControlOuterIntegralMin = -100.0f;
/** @brief PI 外环积分累积上限，单位 g·s（过载倍数·s）。 */
static const float kControlOuterIntegralMax = 100.0f;
/** @brief PID 内环积分累积下限，单位 rad（rad/s 误差乘以 s）。 */
static const float kControlRateIntegralMin = -10.0f;
/** @brief PID 内环积分累积上限，单位 rad（rad/s 误差乘以 s）。 */
static const float kControlRateIntegralMax = 10.0f;
/** @brief PID 内环有符号控制努力下限，归一化范围。 */
static const float kControlSignedEffortMin = -1.0f;
/** @brief PID 内环有符号控制努力上限，归一化范围。 */
static const float kControlSignedEffortMax = 1.0f;
/** @brief 接受的最短控制周期，单位 s（500 us，为 1000 Hz 采样预留抖动余量）。 */
static const float kControlDtMinS = 0.0005f;
/** @brief 接受的最长控制周期，单位 s。 */
static const float kControlDtMaxS = 0.020f;
/** @brief 执行器归一化输出下限。 */
static const float kControlOutputMin = 0.0f;
/** @brief 零有符号控制努力对应的执行器归一化中位值。 */
static const float kControlOutputMidpoint = 0.5f;
/** @brief 执行器归一化输出上限。 */
static const float kControlOutputMax = 1.0f;

/** @brief Roll 执行器输出的对称限幅半宽，围绕 ctrl_midpoint 对称展开。 */
static const float kControlRollOutputHalfRange = 0.1f;

/** @brief 主机控制命令的超时门限，单位 ms。 */
static const uint32_t kHostCommandTimeoutMs = 100U;
/** @brief IMU 数据允许的最大陈旧时间，单位 ms。 */
static const uint32_t kImuStaleTimeoutMs = 20U;

/**
 * @brief PI 控制器配置；所有边界必须为有限数值，且 min 不大于 max。
 */
struct PiControllerConfig {
    /** @brief 比例增益。 */
    float kp;
    /** @brief 积分增益，作用于误差时间积分。 */
    float ki;
    /** @brief 积分量下限，单位随所配置环路的误差单位乘以 s；过载外环为 g·s。 */
    float integral_min;
    /** @brief 积分量上限，单位随所配置环路的误差单位乘以 s；过载外环为 g·s。 */
    float integral_max;
    /** @brief PI 输出下限，单位由控制环路决定。 */
    float output_min;
    /** @brief PI 输出上限，单位由控制环路决定。 */
    float output_max;
};

/**
 * @brief PID 控制器配置；内环输出还必须位于 [-1, 1]，以便映射到执行器范围。
 */
struct PidControllerConfig {
    /** @brief 比例增益。 */
    float kp;
    /** @brief 积分增益，作用于误差时间积分。 */
    float ki;
    /** @brief 微分增益，作用于误差变化率。 */
    float kd;
    /** @brief 积分量下限，单位随所配置环路的误差单位乘以 s；角速度内环为 rad。 */
    float integral_min;
    /** @brief 积分量上限，单位随所配置环路的误差单位乘以 s；角速度内环为 rad。 */
    float integral_max;
    /** @brief 有符号控制努力下限，范围必须不小于 -1。 */
    float output_min;
    /** @brief 有符号控制努力上限，范围必须不大于 1。 */
    float output_max;
};

/**
 * @brief 两级闭环控制器配置，包含物理量换算、环路增益、周期约束和执行器映射范围。
 */
struct ClosedLoopControllerConfig {
    /** @brief 重力加速度，单位 m/s^2，必须大于 0。 */
    float gravity_mps2;
    /** @brief Y 方向（+Y 上向，俯仰）过载指令下限，单位 g。 */
    float n_y_c_min;
    /** @brief Y 方向（+Y 上向，俯仰）过载指令上限，单位 g。 */
    float n_y_c_max;
    /** @brief Z 方向（+Z 右向，偏航）过载指令下限，单位 g。 */
    float n_z_c_min;
    /** @brief Z 方向（+Z 右向，偏航）过载指令上限，单位 g。 */
    float n_z_c_max;
    /** @brief Y 方向（俯仰）过载 PI 外环配置，输出绕 +Z 轴俯仰角速度指令，单位 rad/s。 */
    PiControllerConfig overload_y_pi;
    /** @brief Z 方向（偏航）过载 PI 外环配置，经运动学反向后输出绕 +Y 轴偏航角速度指令，单位 rad/s。 */
    PiControllerConfig overload_z_pi;
    /** @brief Roll 角速度（绕 +X 轴）PID 内环配置，输出为归一化有符号控制努力。 */
    PidControllerConfig rate_r_pid;
    /** @brief Pitch 角速度（绕 +Z 轴）PID 内环配置，输出为归一化有符号控制努力。 */
    PidControllerConfig rate_p_pid;
    /** @brief Yaw 角速度（绕 +Y 轴）PID 内环配置，输出为归一化有符号控制努力。 */
    PidControllerConfig rate_y_pid;
    /** @brief 有效控制周期下限，单位 s，必须大于 0。 */
    float dt_min_s;
    /** @brief 有效控制周期上限，单位 s，必须不小于 dt_min_s。 */
    float dt_max_s;
    /** @brief 执行器归一化输出下限，范围 [0, 1]。 */
    float ctrl_min;
    /** @brief 执行器归一化中位值，必须处于 [ctrl_min, ctrl_max]。 */
    float ctrl_midpoint;
    /** @brief 执行器归一化输出上限，范围 [0, 1]。 */
    float ctrl_max;
    /** @brief Roll 执行器归一化输出下限，必须位于 [ctrl_min, ctrl_midpoint]。 */
    float ctrl_r_min;
    /** @brief Roll 执行器归一化输出上限，必须位于 [ctrl_midpoint, ctrl_max]。 */
    float ctrl_r_max;
};

/**
 * @brief 创建可安全启用的默认闭环控制配置。
 * @return 默认配置；各 PI/PID 增益均为 0，因此有效输入时输出保持 ctrl_midpoint，便于未整定阶段安全联调。
 */
ClosedLoopControllerConfig MakeDefaultClosedLoopControllerConfig();

#endif /* USER_CONFIG_CONTROL_CONFIG_H */
