#pragma once

#include <cstdint>

/**
 * @brief 三轴向量结构体。
 */
struct Vector3f {
	float x;
	float y;
	float z;
};

/**
 * @brief 三回路控制器输出（R/P/Y 合成指令）。
 */
struct RpyCommand {
	float R;
	float P;
	float Y;
};

/**
 * @brief 三回路控制器。
 * @details
 * 外环：接收制导过载指令并进行一阶平滑；
 * 中环：过载误差反馈；
 * 内环：角速度阻尼反馈。
 */
class Control {
public:
	Control();

	/**
	 * @brief 三回路更新。
	 * @param ny_cmd 侧向法向过载指令。
	 * @param nz_cmd 法向过载指令。
	 * @param gyro 陀螺仪角速度（rad/s）。
	 * @param accel 加速度计数据（m/s^2）。
	 * @return RpyCommand R/P/Y 合成指令（单位：deg 等效舵偏角命令）。
	 */
	RpyCommand Update(float ny_cmd, float nz_cmd, const Vector3f& gyro, const Vector3f& accel);

	/**
	 * @brief 舵面分配矩阵。
	 * @details 将 R/P/Y 指令映射到 4 个舵面，并进行整体缩放+单路限幅。
	 * @param R 滚转通道指令。
	 * @param P 俯仰通道指令。
	 * @param Y 偏航通道指令。
	 * @param delta_out 输出数组，长度至少为 4。
	 */
	void Mixer(float R, float P, float Y, float* delta_out) const;

	/**
	 * @brief 设置舵偏角限幅（绝对值，单位 deg）。
	 */
	void SetFinLimit(float fin_limit_deg);

private:
	static float Clamp(float value, float min_value, float max_value);
	static float Abs(float value);

	float ny_ref_;
	float nz_ref_;

	float outer_alpha_;
	float k_ny_;
	float k_nz_;
	float k_roll_rate_;
	float k_pitch_rate_;
	float k_yaw_rate_;

	float cmd_limit_deg_;
	float fin_limit_deg_;
};

