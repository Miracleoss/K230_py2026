#include "alg_control.h"
#include "Config.h"

namespace {

// 控制周期下限，避免 dt 异常小时微分项数值过大。
constexpr float kMinDtSec = 1e-4f;

} // namespace

/**
 * @brief 构造函数。
 * @details
 * 从参数表加载双回路控制所需增益与限制值：
 * - 外环（过载环）PI；
 * - 内环（速率环）P/PD；
 * - 外环积分限幅、外环输出限幅、控制输出限幅与舵机限幅。
 */
Control::Control()
	: int_ny_err_(0.0f),
	  int_nz_err_(0.0f),
	  prev_rate_err_roll_(0.0f),
	  prev_rate_err_pitch_(0.0f),
	  prev_rate_err_yaw_(0.0f),
	  k_gravity_(FcParam::Control::kGravity),
	  k_acc_y_p_(FcParam::Control::kInitPid.yaw.overload_loop.kp),
	  k_acc_y_i_(FcParam::Control::kInitPid.yaw.overload_loop.ki),
	  k_acc_z_p_(FcParam::Control::kInitPid.pitch.overload_loop.kp),
	  k_acc_z_i_(FcParam::Control::kInitPid.pitch.overload_loop.ki),
	  k_rate_roll_p_(FcParam::Control::kInitPid.roll.rate_loop.kp),
	  k_rate_pitch_p_(FcParam::Control::kInitPid.pitch.rate_loop.kp),
	  k_rate_yaw_p_(FcParam::Control::kInitPid.yaw.rate_loop.kp),
	  k_rate_roll_d_(FcParam::Control::kInitPid.roll.rate_loop.kd),
	  k_rate_pitch_d_(FcParam::Control::kInitPid.pitch.rate_loop.kd),
	  k_rate_yaw_d_(FcParam::Control::kInitPid.yaw.rate_loop.kd),
	  i_limit_ny_(FcParam::Control::kInitPid.yaw.overload_loop.i_limit),
	  i_limit_nz_(FcParam::Control::kInitPid.pitch.overload_loop.i_limit),
	  ref_rate_limit_pitch_(FcParam::Control::kInitPid.pitch.overload_loop.out_limit),
	  ref_rate_limit_yaw_(FcParam::Control::kInitPid.yaw.overload_loop.out_limit),
	  cmd_limit_deg_(FcParam::Control::kCmdLimitDeg),
	  fin_limit_deg_(FcParam::Servo::kAngleLimitDeg)
{
}

RpyCommand Control::Update(float ny_cmd, float nz_cmd, const Vector3f& gyro, const Vector3f& accel)
{
	// 离散控制周期
	const float dt = ComputeDtSec();

	// 外环（过载环）：将加速度测量从 m/s^2 转为 g，与 ny_cmd/nz_cmd 统一量纲
	const float ny_meas = accel.y / k_gravity_;
	const float nz_meas = accel.z / k_gravity_;

	// 外环误差：e = n_cmd - n_meas
	const float ny_err = ny_cmd - ny_meas;
	const float nz_err = nz_cmd - nz_meas;

	/*-------------------积分----------------------*/
	// 外环积分更新：I(k) = I(k-1) + e(k)*dt
	int_ny_err_ += ny_err * dt;
	int_nz_err_ += nz_err * dt;
	// 积分限幅
	int_ny_err_ = Clamp(int_ny_err_, -i_limit_ny_, i_limit_ny_);
	int_nz_err_ = Clamp(int_nz_err_, -i_limit_nz_, i_limit_nz_);
	/*-----------------------------------------*/

	// 外环 PID 输出角速度reference
	float yaw_rate_ref = k_acc_y_p_ * ny_err + k_acc_y_i_ * int_ny_err_;
	float pitch_rate_ref = k_acc_z_p_ * nz_err + k_acc_z_i_ * int_nz_err_;

	// 外环输出限幅，约束角速度参考范围
	yaw_rate_ref = Clamp(yaw_rate_ref, -ref_rate_limit_yaw_, ref_rate_limit_yaw_);
	pitch_rate_ref = Clamp(pitch_rate_ref, -ref_rate_limit_pitch_, ref_rate_limit_pitch_);


	// 内环（速率环）误差：roll 目标固定为 0；pitch/yaw 目标来自外环
	const float rate_err_roll = -gyro.x;
	const float rate_err_pitch = pitch_rate_ref - gyro.y;
	const float rate_err_yaw = yaw_rate_ref - gyro.z;

	// 误差离散微分
	const float dr_roll = (rate_err_roll - prev_rate_err_roll_) / dt;
	const float dr_pitch = (rate_err_pitch - prev_rate_err_pitch_) / dt;
	const float dr_yaw = (rate_err_yaw - prev_rate_err_yaw_) / dt;

	// 保存上一拍误差，供下一周期计算微分项
	prev_rate_err_roll_ = rate_err_roll;
	prev_rate_err_pitch_ = rate_err_pitch;
	prev_rate_err_yaw_ = rate_err_yaw;

	RpyCommand out = {};
	// 内环PID
	out.R = k_rate_roll_p_ * rate_err_roll + k_rate_roll_d_ * dr_roll;
	out.P = k_rate_pitch_p_ * rate_err_pitch + k_rate_pitch_d_ * dr_pitch;
	out.Y = k_rate_yaw_p_ * rate_err_yaw + k_rate_yaw_d_ * dr_yaw;

	// 控制器输出限幅，避免进入混控前指令过大
	out.R = Clamp(out.R, -cmd_limit_deg_, cmd_limit_deg_);
	out.P = Clamp(out.P, -cmd_limit_deg_, cmd_limit_deg_);
	out.Y = Clamp(out.Y, -cmd_limit_deg_, cmd_limit_deg_);

	return out;
}

void Control::Mixer(float R, float P, float Y, float* delta_out) const
{
	// 输出缓冲区无效时直接返回。
	if (delta_out == nullptr) {
		return;
	}

	// X 型四舵面混控：滚转为共模，俯仰/偏航为差模。
	float d1 = (P + Y + R)/2.0f;
	float d2 = (P - Y + R)/2.0f;
	float d3 = (-P - Y + R)/2.0f;
	float d4 = (-P + Y + R)/2.0f;

	// 限幅
	// 先找 4 路里绝对值最大的 max_abs
	// 做整体缩放，尽量保持通道比例关系。
	float max_abs = Abs(d1);
	if (Abs(d2) > max_abs) {
		max_abs = Abs(d2);
	}
	if (Abs(d3) > max_abs) {
		max_abs = Abs(d3);
	}
	if (Abs(d4) > max_abs) {
		max_abs = Abs(d4);
	}

	if ((fin_limit_deg_ > 0.0f) && (max_abs > fin_limit_deg_)) {
		float scale = fin_limit_deg_ / max_abs;
		d1 *= scale;
		d2 *= scale;
		d3 *= scale;
		d4 *= scale;
	}

	// 最后做单路硬限幅，确保不超过舵机机械极限。
	delta_out[0] = Clamp(d1, -fin_limit_deg_, fin_limit_deg_);
	delta_out[1] = Clamp(d2, -fin_limit_deg_, fin_limit_deg_);
	delta_out[2] = Clamp(d3, -fin_limit_deg_, fin_limit_deg_);
	delta_out[3] = Clamp(d4, -fin_limit_deg_, fin_limit_deg_);
}

/**
 * @brief 更新舵面限幅。
 * @param fin_limit_deg 舵偏绝对值上限（deg），需大于 0。
 */
void Control::SetFinLimit(float fin_limit_deg)
{
	if (fin_limit_deg <= 0.0f) {
		return;
	}

	fin_limit_deg_ = fin_limit_deg;
}

/**
 * @brief 标量限幅。
 */
float Control::Clamp(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}

	if (value > max_value) {
		return max_value;
	}

	return value;
}

/**
 * @brief 绝对值计算。
 */
float Control::Abs(float value)
{
	return (value < 0.0f) ? -value : value;
}

/**
 * @brief 获取离散控制周期（秒）。
 * @details
 * 使用调度周期 HwConfig::App::kFlightPeriodMs 换算为秒，并施加最小值保护。
 */
float Control::ComputeDtSec()
{
	const float dt = static_cast<float>(HwConfig::App::kFlightPeriodMs) * 1e-3f;
	return (dt > kMinDtSec) ? dt : kMinDtSec;
}

