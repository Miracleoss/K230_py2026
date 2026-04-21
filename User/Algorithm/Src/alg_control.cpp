#include "alg_control.h"
#include "Config.h"

namespace {

} // namespace

Control::Control()
	: ny_ref_(0.0f),
	  nz_ref_(0.0f),
	  outer_alpha_(FcParam::Control::kOuterAlpha),
	  k_ny_(FcParam::Control::kInitPid.yaw.overload_loop.kp),
	  k_nz_(FcParam::Control::kInitPid.pitch.overload_loop.kp),
	  k_roll_rate_(FcParam::Control::kInitPid.roll.rate_loop.kp),
	  k_pitch_rate_(FcParam::Control::kInitPid.pitch.rate_loop.kp),
	  k_yaw_rate_(FcParam::Control::kInitPid.yaw.rate_loop.kp),
	  cmd_limit_deg_(FcParam::Control::kCmdLimitDeg),
	  fin_limit_deg_(FcParam::Servo::kAngleLimitDeg)
{
}

RpyCommand Control::Update(float ny_cmd, float nz_cmd, const Vector3f& gyro, const Vector3f& accel)
{
	// 外环：对制导指令做一阶平滑，降低突变导致的舵面冲击。
	ny_ref_ += outer_alpha_ * (ny_cmd - ny_ref_);
	nz_ref_ += outer_alpha_ * (nz_cmd - nz_ref_);

	// 中环：过载反馈（测量值由 m/s^2 转换为 g）。
	float ny_meas = accel.y / FcParam::Control::kGravity;
	float nz_meas = accel.z / FcParam::Control::kGravity;

	float ny_err = ny_ref_ - ny_meas;
	float nz_err = nz_ref_ - nz_meas;

	float y_mid = k_ny_ * ny_err;
	float p_mid = k_nz_ * nz_err;

	// 内环：角速度阻尼反馈。
	RpyCommand out = {};
	out.R = -k_roll_rate_ * gyro.x;
	out.P = p_mid - k_pitch_rate_ * gyro.y;
	out.Y = y_mid - k_yaw_rate_ * gyro.z;

	out.R = Clamp(out.R, -cmd_limit_deg_, cmd_limit_deg_);
	out.P = Clamp(out.P, -cmd_limit_deg_, cmd_limit_deg_);
	out.Y = Clamp(out.Y, -cmd_limit_deg_, cmd_limit_deg_);

	return out;
}

void Control::Mixer(float R, float P, float Y, float* delta_out) const
{
	if (delta_out == nullptr) {
		return;
	}

	// 标准飞镖四舵面分配：滚转为共模，俯仰/偏航为差模。
	float d1 = (P + Y + R)/2.0f;
	float d2 = (P - Y + R)/2.0f;
	float d3 = (-P - Y + R)/2.0f;
	float d4 = (-P + Y + R)/2.0f;

	// 先做整体缩放，尽量保持通道比例关系。
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

	// 最后做单路硬限幅，保证机械安全。
	delta_out[0] = Clamp(d1, -fin_limit_deg_, fin_limit_deg_);
	delta_out[1] = Clamp(d2, -fin_limit_deg_, fin_limit_deg_);
	delta_out[2] = Clamp(d3, -fin_limit_deg_, fin_limit_deg_);
	delta_out[3] = Clamp(d4, -fin_limit_deg_, fin_limit_deg_);
}

void Control::SetFinLimit(float fin_limit_deg)
{
	if (fin_limit_deg <= 0.0f) {
		return;
	}

	fin_limit_deg_ = fin_limit_deg;
}

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

float Control::Abs(float value)
{
	return (value < 0.0f) ? -value : value;
}

