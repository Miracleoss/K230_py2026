#pragma once

#ifdef __cplusplus

namespace FcParam {

struct PidParam {
	float kp;
	float ki;
	float kd;
	float i_limit;
	float out_limit;
};

struct AxisLoopParam {
	PidParam overload_loop;
	PidParam rate_loop;
};

struct ThreeLoopPidParam {
	AxisLoopParam roll;
	AxisLoopParam pitch;
	AxisLoopParam yaw;
};

namespace Servo {

static constexpr float kCenterPulseUs = 1500.0f;
static constexpr float kPulseSpanUs = 500.0f;
static constexpr float kPulseMinUs = 1000.0f;
static constexpr float kPulseMaxUs = 2000.0f;
static constexpr float kMapAngleMinDeg = -45.0f;
static constexpr float kMapAngleMaxDeg = 45.0f;
static constexpr float kAngleLimitDeg = 25.0f;

} // namespace Servo

namespace Control {

static constexpr float kGravity = 9.80665f;
static constexpr float kOuterAlpha = 0.20f;
static constexpr float kCmdLimitDeg = 30.0f;
static constexpr float kGuidanceCmdLimitG = 8.0f;

// 预留三回路 PID 初始参数（可在调参阶段在线或离线覆盖）
static constexpr ThreeLoopPidParam kInitPid = {
	// Roll
	{
		{0.0f, 0.00f, 0.0f, 2.0f, 30.0f}, // overload loop
		{0.0f, 0.00f, 0.0f, 1.5f, 25.0f}, // rate loop
	},
	// Pitch
	{
		{1.40f, 0.00f, 0.03f, 2.0f, 35.0f}, // overload loop
		{0.65f, 0.00f, 0.01f, 1.5f, 30.0f}, // rate loop
	},
	// Yaw
	{
		{1.00f, 0.00f, 0.02f, 2.0f, 30.0f}, // overload loop
		{0.60f, 0.00f, 0.01f, 1.5f, 25.0f}, // rate loop
	},
};

} // namespace Control

namespace ImuInstall {

// 传感器坐标 -> 机体坐标方向余弦矩阵
// 当前定义: BMI088 X/Y/Z 分别对应 飞镖机体 X/Y/Z
static constexpr float kSensorToBody[3][3] = {
	{1.0f, 0.0f, 0.0f},
	{0.0f, 1.0f, 0.0f},
	{0.0f, 0.0f, 1.0f},
};

} // namespace ImuInstall

} // namespace FcParam

#endif

