#include "Algorithm/alg_closed_loop_controller.h"

#include <float.h>

/* 基础数值 helper：在不依赖运行库分类函数的前提下，拒绝超出 float 有限范围的输入。 */
static bool IsFiniteFloat(float value)
{
    return value <= FLT_MAX && value >= -FLT_MAX;
}

/* 通用限幅 helper：将控制状态和输出限制在已校验的闭区间内。 */
static float ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/* 解析样本有效微秒时间戳：优先使用硬件 1 us 时间戳，未填充时回退为毫秒时间戳折算值。 */
static uint64_t ResolveSampleTimestampUs(const ImuData& imu)
{
    if (imu.sample_timestamp_us != 0ULL) {
        return imu.sample_timestamp_us;
    }
    return static_cast<uint64_t>(imu.sample_timestamp_ms) * 1000ULL;
}

/* 校验 PI 配置的有限性和边界顺序，防止控制计算在非法配置下继续执行。 */
static bool IsPiConfigValid(const PiControllerConfig& config)
{
    return IsFiniteFloat(config.kp) &&
           IsFiniteFloat(config.ki) &&
           IsFiniteFloat(config.integral_min) &&
           IsFiniteFloat(config.integral_max) &&
           IsFiniteFloat(config.output_min) &&
           IsFiniteFloat(config.output_max) &&
           config.integral_min <= config.integral_max &&
           config.output_min <= config.output_max;
}

/* 校验 PID 配置，并额外确保有符号努力可映射至 [-1, 1] 的执行器域。 */
static bool IsPidConfigValid(const PidControllerConfig& config)
{
    return IsFiniteFloat(config.kp) &&
           IsFiniteFloat(config.ki) &&
           IsFiniteFloat(config.kd) &&
           IsFiniteFloat(config.integral_min) &&
           IsFiniteFloat(config.integral_max) &&
           IsFiniteFloat(config.output_min) &&
           IsFiniteFloat(config.output_max) &&
           config.integral_min <= config.integral_max &&
           config.output_min >= -1.0f &&
           config.output_max <= 1.0f &&
           config.output_min <= config.output_max;
}

/* 汇总校验控制器全部物理常量、环路配置、周期范围和输出映射范围。 */
static bool IsControllerConfigValid(const ClosedLoopControllerConfig& config)
{
    return IsFiniteFloat(config.gravity_mps2) && config.gravity_mps2 > 0.0f &&
           IsFiniteFloat(config.n_y_c_min) &&
           IsFiniteFloat(config.n_y_c_max) &&
           IsFiniteFloat(config.n_z_c_min) &&
           IsFiniteFloat(config.n_z_c_max) &&
           config.n_y_c_min <= config.n_y_c_max &&
           config.n_z_c_min <= config.n_z_c_max &&
           IsPiConfigValid(config.overload_y_pi) &&
           IsPiConfigValid(config.overload_z_pi) &&
           IsPidConfigValid(config.rate_r_pid) &&
           IsPidConfigValid(config.rate_p_pid) &&
           IsPidConfigValid(config.rate_y_pid) &&
           IsFiniteFloat(config.dt_min_s) &&
           IsFiniteFloat(config.dt_max_s) &&
           config.dt_min_s > 0.0f &&
           config.dt_min_s <= config.dt_max_s &&
           IsFiniteFloat(config.ctrl_min) &&
           IsFiniteFloat(config.ctrl_midpoint) &&
           IsFiniteFloat(config.ctrl_max) &&
           config.ctrl_min >= 0.0f &&
           config.ctrl_min <= config.ctrl_midpoint &&
           config.ctrl_midpoint <= config.ctrl_max &&
           config.ctrl_max <= 1.0f &&
           IsFiniteFloat(config.ctrl_r_min) &&
           IsFiniteFloat(config.ctrl_r_max) &&
           config.ctrl_min <= config.ctrl_r_min &&
           config.ctrl_r_min <= config.ctrl_midpoint &&
           config.ctrl_midpoint <= config.ctrl_r_max &&
           config.ctrl_r_max <= config.ctrl_max;
}

/*
 * PI 外环更新：先计算受积分边界约束的候选积分，再进行条件积分 anti-windup。
 * 若输出已向误差同向饱和，则保留旧积分量，避免积分继续推动饱和；最终输出始终限幅。
 */
static float UpdatePi(
    const PiControllerConfig& config,
    float error,
    float dt_s,
    float& integral)
{
    const float candidate_integral = ClampFloat(
        integral + error * dt_s,
        config.integral_min,
        config.integral_max);
    float unbounded_output = config.kp * error + config.ki * candidate_integral;

    const float integral_drive = config.ki * error;
    const bool saturating_high = unbounded_output > config.output_max && integral_drive > 0.0f;
    const bool saturating_low = unbounded_output < config.output_min && integral_drive < 0.0f;
    if (!saturating_high && !saturating_low) {
        integral = candidate_integral;
    } else {
        unbounded_output = config.kp * error + config.ki * integral;
    }

    return ClampFloat(unbounded_output, config.output_min, config.output_max);
}

/*
 * PID 内环更新：首个有效样本不计算微分项，随后使用相邻误差差分。
 * 与 PI 一致，输出同向饱和时冻结积分量实现 anti-windup，同时仍更新误差历史供下一周期求导。
 */
static float UpdatePid(
    const PidControllerConfig& config,
    float error,
    float dt_s,
    float& integral,
    float& previous_error,
    bool& has_previous_error)
{
    const float derivative = has_previous_error ? (error - previous_error) / dt_s : 0.0f;
    const float candidate_integral = ClampFloat(
        integral + error * dt_s,
        config.integral_min,
        config.integral_max);
    float unbounded_output =
        config.kp * error + config.ki * candidate_integral + config.kd * derivative;

    const float integral_drive = config.ki * error;
    const bool saturating_high = unbounded_output > config.output_max && integral_drive > 0.0f;
    const bool saturating_low = unbounded_output < config.output_min && integral_drive < 0.0f;
    if (!saturating_high && !saturating_low) {
        integral = candidate_integral;
    } else {
        unbounded_output = config.kp * error + config.ki * integral + config.kd * derivative;
    }

    previous_error = error;
    has_previous_error = true;
    return ClampFloat(unbounded_output, config.output_min, config.output_max);
}

/*
 * 将 [-1, 1] 的有符号控制努力映射到配置的非对称执行器范围。
 * 正、负方向分别以 ctrl_midpoint 为基准缩放，保持零努力对应中位值。
 */
static float MapSignedEffortToControl(
    float signed_effort,
    const ClosedLoopControllerConfig& config)
{
    float control = config.ctrl_midpoint;
    if (signed_effort >= 0.0f) {
        control += signed_effort * (config.ctrl_max - config.ctrl_midpoint);
    } else {
        control += signed_effort * (config.ctrl_midpoint - config.ctrl_min);
    }
    return ClampFloat(control, config.ctrl_min, config.ctrl_max);
}

/*
 * 写入失效快照：测量和指令清零，三个执行器取有限的配置中位值，并标记为无效。
 * 有效配置下该中位值位于输出范围内；若中位值有限但越界，代码仍原样写入；仅当中位值非有限时回退为 0.5f。
 */
static void SetNeutralOutput(
    const ClosedLoopControllerConfig& config,
    ClosedLoopControlOutput& output)
{
    output.n_y_meas = 0.0f;
    output.n_z_meas = 0.0f;
    output.omega_x_c = 0.0f;
    output.omega_y_c = 0.0f;
    output.omega_z_c = 0.0f;
    output.ctrl_R = IsFiniteFloat(config.ctrl_midpoint) ? config.ctrl_midpoint : 0.5f;
    output.ctrl_P = output.ctrl_R;
    output.ctrl_Y = output.ctrl_R;
    output.valid = false;
}

/*
 * 默认配置采用零 PI/PID 增益：即使数据有效也不会施加偏离中位值的控制努力。
 * 这为增益尚未整定或联调阶段提供可预测的安全基线；积分与输出边界仍保持完整校验。
 */
ClosedLoopControllerConfig MakeDefaultClosedLoopControllerConfig()
{
    ClosedLoopControllerConfig config;
    config.gravity_mps2 = kControlGravityMps2;
    config.n_y_c_min = kControlOverloadCommandMinG;
    config.n_y_c_max = kControlOverloadCommandMaxG;
    config.n_z_c_min = kControlOverloadCommandMinG;
    config.n_z_c_max = kControlOverloadCommandMaxG;

    //加速度环 Y轴 PI 配置（Pitch过载指令）
    config.overload_y_pi.kp = 0.0f;
    config.overload_y_pi.ki = 0.0f;
    config.overload_y_pi.integral_min = kControlOuterIntegralMin;
    config.overload_y_pi.integral_max = kControlOuterIntegralMax;
    config.overload_y_pi.output_min = kControlRateCommandMinRadps;
    config.overload_y_pi.output_max = kControlRateCommandMaxRadps;
    // config.overload_z_pi = config.overload_y_pi;//Z轴这里暂时使用相同的 PI 配置，后续可根据需要调整
    //加速度环 Z轴 PI 配置（Yaw过载指令） 
    config.overload_z_pi.kp = 3.0f;
	// config.overload_z_pi.kp = 0.0f;
    config.overload_z_pi.ki = 6.0f;
    // config.overload_z_pi.ki = 0.0f;
    config.overload_z_pi.integral_min = kControlOuterIntegralMin;
    config.overload_z_pi.integral_max = kControlOuterIntegralMax;
    config.overload_z_pi.output_min = kControlRateCommandMinRadps;
    config.overload_z_pi.output_max = kControlRateCommandMaxRadps;

    //角速度环 Roll PID 配置
    config.rate_r_pid.kp = 0.5f;
    config.rate_r_pid.ki = 0.0f;
    config.rate_r_pid.kd = 0.0f;
    config.rate_r_pid.integral_min = kControlRateIntegralMin;
    config.rate_r_pid.integral_max = kControlRateIntegralMax;
    config.rate_r_pid.output_min = kControlSignedEffortMin;
    config.rate_r_pid.output_max = kControlSignedEffortMax;

    //角速度环 Pitch PID 配置
    config.rate_p_pid.kp = 0.0f;
    config.rate_p_pid.ki = 0.0f;
    config.rate_p_pid.kd = 0.0f;
    config.rate_p_pid.integral_min = kControlRateIntegralMin;
    config.rate_p_pid.integral_max = kControlRateIntegralMax;
    config.rate_p_pid.output_min = kControlSignedEffortMin;
    config.rate_p_pid.output_max = kControlSignedEffortMax;

    //角速度环 Yaw PID 配置（机体系 +X前/+Y上/+Z右 下绕 +Y 轴偏航，正增益构成负反馈）
    config.rate_y_pid.kp = 1.4f;
    // config.rate_y_pid.kp = 0.0f;
	config.rate_y_pid.ki = 0.0f;
    config.rate_y_pid.kd = 0.0f;
    config.rate_y_pid.integral_min = kControlRateIntegralMin;
    config.rate_y_pid.integral_max = kControlRateIntegralMax;
    config.rate_y_pid.output_min = kControlSignedEffortMin;
    config.rate_y_pid.output_max = kControlSignedEffortMax;

    // 控制周期和输出范围配置
    config.dt_min_s = kControlDtMinS;
    config.dt_max_s = kControlDtMaxS;
    config.ctrl_min = kControlOutputMin;
    config.ctrl_midpoint = kControlOutputMidpoint;
    config.ctrl_max = kControlOutputMax;

    // Roll 最终输出限幅：围绕中位值 ±0.1，即 [0.4, 0.6]，避免 Roll 阻尼环过度占用舵面行程。
    config.ctrl_r_min = kControlOutputMidpoint - kControlRollOutputHalfRange;
    config.ctrl_r_max = kControlOutputMidpoint + kControlRollOutputHalfRange;
    return config;
}
 
ClosedLoopControlOutput::ClosedLoopControlOutput()
    : n_y_meas(0.0f),
      n_z_meas(0.0f),
      omega_x_c(0.0f),
      omega_y_c(0.0f),
      omega_z_c(0.0f),
      ctrl_R(kControlOutputMidpoint),
      ctrl_P(kControlOutputMidpoint),
      ctrl_Y(kControlOutputMidpoint),
      valid(false)
{
}

Class_ClosedLoopController::Class_ClosedLoopController()
    : config_(MakeDefaultClosedLoopControllerConfig())
{
    Reset();
}

Class_ClosedLoopController::Class_ClosedLoopController(
    const ClosedLoopControllerConfig& config)
    : config_(config)
{
    Reset();
}

void Class_ClosedLoopController::Reset()
{
    /* 第一块：清空过载 PI 外环积分状态。 */
    overload_y_state_.integral = 0.0f;
    overload_z_state_.integral = 0.0f;

    /* 第二块：清空角速度 PID 内环积分和微分历史。 */
    rate_r_state_.integral = 0.0f;
    rate_r_state_.previous_error = 0.0f;
    rate_r_state_.has_previous_error = false;
    rate_p_state_ = rate_r_state_;
    rate_y_state_ = rate_r_state_;

    /* 第三块：撤销样本门控记录，使复位后的首个有效样本能够重新进入控制环。 */
    last_sample_timestamp_us_ = 0ULL;
    last_data_ready_seq_ = 0U;
    has_last_sample_ = false;
}

ResultCode Class_ClosedLoopController::Update(
    float n_y_c,
    float n_z_c,
    const ImuData& imu,
    float dt_s,
    ClosedLoopControlOutput& output,
    bool enable_roll,
    bool enable_pitch_yaw)
{
    /* 失败路径先写入失效快照；仅有效配置能保证其中位控制量处于配置范围，随后所有校验通过后才覆盖为有效结果。 */
    SetNeutralOutput(config_, output);

    /* 输入校验同时覆盖指令、周期和参与计算的所有 IMU 浮点量，拒绝非有限数值。 */
    const bool finite_input =
        IsFiniteFloat(n_y_c) &&
        IsFiniteFloat(n_z_c) &&
        IsFiniteFloat(dt_s) &&
        IsFiniteFloat(imu.specific_force_y_mps2) &&
        IsFiniteFloat(imu.specific_force_z_mps2) &&
        IsFiniteFloat(imu.gyro_x_radps) &&
        IsFiniteFloat(imu.gyro_y_radps) &&
        IsFiniteFloat(imu.gyro_z_radps);
    /* 周期约束保证积分和离散微分均在配置的设计采样范围内运行。 */
    const bool valid_dt = dt_s >= config_.dt_min_s && dt_s <= config_.dt_max_s;
    const uint64_t sample_timestamp_us = ResolveSampleTimestampUs(imu);
    /* 样本门控要求微秒时间戳或 data_ready_seq 至少一个变化，避免对同一 IMU 样本重复积分。 */
    const bool new_sample =
        !has_last_sample_ ||
        (sample_timestamp_us != last_sample_timestamp_us_ ||
         imu.data_ready_seq != last_data_ready_seq_);
    /* 时间戳不得回退；回退意味着样本时序不可用于可靠的离散控制。 */
    const bool timestamp_progress_valid =
        !has_last_sample_ || sample_timestamp_us >= last_sample_timestamp_us_;

    if (!IsControllerConfigValid(config_) ||
        !finite_input ||
        !imu.valid ||
        !valid_dt ||
        !timestamp_progress_valid ||
        !new_sample) {
        Reset();
        return ResultCode::InvalidData;
    }

    /* 将 IMU 比力换算为 g，作为过载外环的实际反馈。 */
    const float f_y = imu.specific_force_y_mps2;
    const float f_z = imu.specific_force_z_mps2;
    output.n_y_meas = f_y / config_.gravity_mps2;
    output.n_z_meas = f_z / config_.gravity_mps2;

    /* Roll 轴控制：外环指令固定为 0，内环角速度 PID 根据 enable_roll 决定是否闭环 */
    output.omega_x_c = 0.0f;
    if (enable_roll) {
        const float omega_x_meas = imu.gyro_x_radps;
        const float e_omega_x = output.omega_x_c - omega_x_meas;
        const float signed_R = UpdatePid(
            config_.rate_r_pid,
            e_omega_x,
            dt_s,
            rate_r_state_.integral,
            rate_r_state_.previous_error,
            rate_r_state_.has_previous_error);
        output.ctrl_R = ClampFloat(
            MapSignedEffortToControl(signed_R, config_),
            config_.ctrl_r_min,
            config_.ctrl_r_max);
    } else {
        rate_r_state_.integral = 0.0f;
        rate_r_state_.previous_error = 0.0f;
        rate_r_state_.has_previous_error = false;
        output.ctrl_R = config_.ctrl_midpoint;
    }

    /* Pitch/Yaw 轴控制：根据 enable_pitch_yaw 决定是否执行过载 PI 与角速度 PID 闭环 */
    if (enable_pitch_yaw) {
        /*
         * 外环 PI（机体系 +X 前、+Y 上、+Z 右）：
         * - Pitch 俯仰：+Y 上向过载误差 e_ny 产生绕 +Z 右轴的俯仰角速度指令 omega_z_c（a_y = +omega_z * V）；
         * - Yaw 偏航：+Z 右向过载误差 e_nz 产生绕 +Y 上轴的偏航角速度指令 omega_y_c（a_z = -omega_y * V，带运动学负号）。
         */
        const float bounded_n_y_c = ClampFloat(n_y_c, config_.n_y_c_min, config_.n_y_c_max);
        const float bounded_n_z_c = ClampFloat(n_z_c, config_.n_z_c_min, config_.n_z_c_max);
        const float e_ny = bounded_n_y_c - output.n_y_meas;
        const float e_nz = bounded_n_z_c - output.n_z_meas;
        // const float e_nz = bounded_n_z_c - 0.0f;//这里暂时测试

        output.omega_z_c = UpdatePi(
            config_.overload_y_pi,
            e_ny,
            dt_s,
            overload_y_state_.integral);
        output.omega_y_c = 0.0f - UpdatePi(//这里注意运动学反向，Yaw过载指令输出绕 +Y 轴角速度指令 omega_y_c = -a_z / V
            config_.overload_z_pi,
            e_nz,
            dt_s,
            overload_z_state_.integral);

        /*
         * 内环 PID：
         * - Pitch 内环跟踪绕 +Z 轴俯仰角速度（omega_z_c vs gyro_z_radps），输出 ctrl_P；
         * - Yaw 内环跟踪绕 +Y 轴偏航角速度（omega_y_c vs gyro_y_radps），输出 ctrl_Y。
         */
        const float omega_y_meas = imu.gyro_y_radps;
        const float omega_z_meas = imu.gyro_z_radps;
        const float e_omega_pitch = output.omega_z_c - omega_z_meas;
        const float e_omega_yaw = output.omega_y_c - omega_y_meas;

        const float signed_P = UpdatePid(
            config_.rate_p_pid,
            e_omega_pitch,
            dt_s,
            rate_p_state_.integral,
            rate_p_state_.previous_error,
            rate_p_state_.has_previous_error);
        const float signed_Y = UpdatePid(
            config_.rate_y_pid,
            e_omega_yaw,
            dt_s,
            rate_y_state_.integral,
            rate_y_state_.previous_error,
            rate_y_state_.has_previous_error);

        output.ctrl_P = MapSignedEffortToControl(signed_P, config_);
        output.ctrl_Y = MapSignedEffortToControl(signed_Y, config_);
    } else {
        overload_y_state_.integral = 0.0f;
        overload_z_state_.integral = 0.0f;
        rate_p_state_.integral = 0.0f;
        rate_p_state_.previous_error = 0.0f;
        rate_p_state_.has_previous_error = false;
        rate_y_state_.integral = 0.0f;
        rate_y_state_.previous_error = 0.0f;
        rate_y_state_.has_previous_error = false;
        output.omega_y_c = 0.0f;
        output.omega_z_c = 0.0f;
        output.ctrl_P = config_.ctrl_midpoint;
        output.ctrl_Y = config_.ctrl_midpoint;
    }

    output.valid = true;

    /* 仅在整次更新成功后提交样本门控记录，确保失败样本不会污染后续判定。 */
    last_sample_timestamp_us_ = sample_timestamp_us;
    last_data_ready_seq_ = imu.data_ready_seq;
    has_last_sample_ = true;
    return ResultCode::Ok;
}
