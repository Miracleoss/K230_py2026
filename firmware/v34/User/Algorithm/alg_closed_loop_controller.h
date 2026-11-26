#ifndef USER_ALGORITHM_ALG_CLOSED_LOOP_CONTROLLER_H
#define USER_ALGORITHM_ALG_CLOSED_LOOP_CONTROLLER_H

#include <stdint.h>

#include "Common/common_types.h"
#include "Common/imu_data.h"
#include "Config/control_config.h"

/**
 * @brief 闭环控制器在一次 Update 后给出的测量、指令和执行器输出快照。
 *
 * 当 valid 为 false 时，其余字段仅为失效快照，调用方不得将其视为有效控制命令。
 * 失效时测得过载和角速度指令为零；控制量取有限的 config.ctrl_midpoint，只有该值非有限时才回退为 0.5f，
 * 因而有限但越界的配置中位值会被原样写入，不能据此断言输出处于配置范围内。
 */
struct ClosedLoopControlOutput {
    /** @brief Y 方向（+Y 上向，俯仰）测得过载，单位 g。 */
    float n_y_meas;
    /** @brief Z 方向（+Z 右向，偏航）测得过载，单位 g。 */
    float n_z_meas;
    /** @brief Roll 角速度指令（绕 +X 前向轴），单位 rad/s；当前外环固定为 0。 */
    float omega_x_c;
    /** @brief Yaw 角速度指令（绕 +Y 上向轴，机头向左为正），单位 rad/s；受配置的 PI 输出范围限制。 */
    float omega_y_c;
    /** @brief Pitch 角速度指令（绕 +Z 右向轴，机头向上为正），单位 rad/s；受配置的 PI 输出范围限制。 */
    float omega_z_c;
    /** @brief Roll 控制量，归一化范围为配置的 [ctrl_r_min, ctrl_r_max]。 */
    float ctrl_R;
    /** @brief Pitch 控制量，归一化范围为配置的 [ctrl_min, ctrl_max]。 */
    float ctrl_P;
    /** @brief Yaw 控制量，归一化范围为配置的 [ctrl_min, ctrl_max]。 */
    float ctrl_Y;
    /** @brief true 表示本次 Update 成功并且其余字段有效；false 表示失效快照，不可作为有效控制命令使用。 */
    bool valid;

    /** @brief 构造默认安全中性输出，valid 初始为 false。 */
    ClosedLoopControlOutput();
};

/**
 * @brief 两级闭环控制器：过载 PI 外环产生角速度指令，角速度 PID 内环产生控制量。
 */
class Class_ClosedLoopController {
public:
    /** @brief 使用 MakeDefaultClosedLoopControllerConfig 的默认配置构造，并复位全部状态。 */
    Class_ClosedLoopController();
    /**
     * @brief 使用指定配置构造，并复位全部积分、微分与样本门控状态。
     * @param config 控制器配置；将在每次 Update 时校验其数值范围。
     */
    explicit Class_ClosedLoopController(const ClosedLoopControllerConfig& config);

    /**
     * @brief 清除 PI/PID 积分、PID 历史误差及 IMU 样本门控记录。
     */
    void Reset();
    /**
     * @brief 基于一份新的 IMU 样本执行一次两级闭环控制更新。
     * @param n_y_c Y 方向过载指令，单位 g；超出配置 [n_y_c_min, n_y_c_max] 时会被限幅。
     * @param n_z_c Z 方向过载指令，单位 g；超出配置 [n_z_c_min, n_z_c_max] 时会被限幅。
     * @param imu 有效的 IMU 数据；比力单位 m/s^2、角速度单位 rad/s，并须为新样本。
     * @param dt_s 控制周期，单位 s，必须处于配置 [dt_min_s, dt_max_s] 内。
     * @param output 输出快照；失败时测得过载和角速度指令置零，控制量取有限的 config.ctrl_midpoint（仅非有限时回退 0.5f），并将 valid 设为 false；调用方不得将此快照作为有效命令使用。
     * @param enable_roll 是否使能 Roll 轴闭环（已发射时为 true，未发射时为 false）。
     * @param enable_pitch_yaw 是否使能 Pitch/Yaw 轴闭环（已识别绿灯时为 true）。
     * @return ResultCode::Ok 表示更新成功；ResultCode::InvalidData 表示配置、输入、时间戳或样本门控校验失败，且内部状态已复位。
     */
    ResultCode Update(
        float n_y_c,
        float n_z_c,
        const ImuData& imu,
        float dt_s,
        ClosedLoopControlOutput& output,
        bool enable_roll = true,
        bool enable_pitch_yaw = true);

private:
    /** @brief PI 外环状态；积分量单位为过载误差乘以秒，即 g·s（过载倍数·s）。 */
    struct PiState {
        /** @brief 已限幅的过载误差时间积分，单位 g·s（过载倍数·s）。 */
        float integral;
    };

    /** @brief PID 内环状态，包含积分量与离散微分所需的上一误差。 */
    struct PidState {
        /** @brief 已限幅的角速度误差时间积分，单位 rad。 */
        float integral;
        /** @brief 上一次角速度误差，单位 rad/s。 */
        float previous_error;
        /** @brief 是否已有 previous_error；首个样本的微分项为 0。 */
        bool has_previous_error;
    };

    /** @brief 运行时控制器配置；非法配置会使 Update 返回 InvalidData。 */
    ClosedLoopControllerConfig config_;
    /** @brief Y 过载 PI 外环积分状态。 */
    PiState overload_y_state_;
    /** @brief Z 过载 PI 外环积分状态。 */
    PiState overload_z_state_;
    /** @brief Roll 角速度 PID 内环状态。 */
    PidState rate_r_state_;
    /** @brief Pitch 角速度 PID 内环状态。 */
    PidState rate_p_state_;
    /** @brief Yaw 角速度 PID 内环状态。 */
    PidState rate_y_state_;
    /** @brief 最近一次已接受 IMU 样本的有效微秒时间戳，单位 us。 */
    uint64_t last_sample_timestamp_us_;
    /** @brief 最近一次已接受 IMU 样本的数据就绪序号。 */
    uint32_t last_data_ready_seq_;
    /** @brief 是否已有已接受样本；用于首样本放行和重复样本门控。 */
    bool has_last_sample_;
};

#endif /* USER_ALGORITHM_ALG_CLOSED_LOOP_CONTROLLER_H */
