#pragma once

#include <cstdint>

/**
 * @brief 三轴浮点向量。
 * @details
 * 统一用于陀螺仪与加速度计输入。
 * - 对于 gyro：x/y/z 分别对应机体系滚转/俯仰/偏航角速度。
 * - 对于 accel：x/y/z 分别对应机体系线加速度。
 */
struct Vector3f {
	/** 机体系 X 轴分量。 */
	float x;
	/** 机体系 Y 轴分量。 */
	float y;
	/** 机体系 Z 轴分量。 */
	float z;
};

/**
 * @brief 控制器三通道输出。
 * @details
 * 输出为混控前的 R/P/Y 指令：
 * - R：滚转阻尼输出；
 * - P：俯仰通道舵偏指令；
 * - Y：偏航通道舵偏指令。
 */
struct RpyCommand {
	/** 滚转通道输出。 */
	float R;
	/** 俯仰通道输出。 */
	float P;
	/** 偏航通道输出。 */
	float Y;
};

/**
 * @brief 双回路嵌套控制器。
 * @details
 * 控制结构如下：
 * 1) 外环（过载环）：输入 ny_cmd/nz_cmd 与加速度测量，PI 输出角速度参考；
 * 2) 内环（速率环）：输入角速度参考与陀螺测量，P/PD 输出舵机偏移；
 * 3) Roll 通道：无外部指令，目标角速度固定为 0，仅做阻尼。
 */
class Control {
public:
	/**
	 * @brief 构造控制器并加载默认参数。
	 * @details 参数来源于 FcParam::Control，包含外环 PI、内环 P/PD、限幅配置。
	 */
	Control();

	/**
	 * @brief 双回路嵌套控制更新。
	 * @param ny_cmd 侧向法向过载指令（建议单位 g）。
	 * @param nz_cmd 法向过载指令（建议单位 g）。
	 * @param gyro 陀螺仪角速度（rad/s）。
	 * @param accel 加速度计测量（m/s^2）。
	 * @return RpyCommand R/P/Y 合成指令（送入 Mixer 前）。
	 * @note
	 * 实现中会将 accel.y/accel.z 按重力常数换算为 g，与 ny_cmd/nz_cmd 保持同量纲。
	 */
	RpyCommand Update(float ny_cmd, float nz_cmd, const Vector3f& gyro, const Vector3f& accel);

	/**
	 * @brief 舵面分配矩阵。
	 * @details
	 * 保持 X 型四舵面布局，将 R/P/Y 映射为 4 路舵偏：
	 * - 先计算原始分配值；
	 * - 再进行整体等比缩放（保持通道比例）；
	 * - 最后执行单路硬限幅（保证机械安全）。
	 * @param R 滚转通道指令。
	 * @param P 俯仰通道指令。
	 * @param Y 偏航通道指令。
	 * @param delta_out 输出数组，长度至少为 4。
	 */
	void Mixer(float R, float P, float Y, float* delta_out) const;

	/**
	 * @brief 设置舵偏角限幅（绝对值，单位 deg）。
	 * @param fin_limit_deg 舵面极限角（正值）。
	 */
	void SetFinLimit(float fin_limit_deg);

private:
	/** @brief 数值限幅工具函数。 */
	static float Clamp(float value, float min_value, float max_value);
	/** @brief 绝对值工具函数。 */
	static float Abs(float value);
	/** @brief 计算控制周期（秒）。 */
	static float ComputeDtSec();

	/** 外环 PI：ny 误差积分状态（与 ny_cmd 同单位，默认 g）。 */
	float int_ny_err_;
	/** 外环 PI：nz 误差积分状态（与 nz_cmd 同单位，默认 g）。 */
	float int_nz_err_;

	/** 内环 D 项用：roll 速率误差上一拍。 */
	float prev_rate_err_roll_;
	/** 内环 D 项用：pitch 速率误差上一拍。 */
	float prev_rate_err_pitch_;
	/** 内环 D 项用：yaw 速率误差上一拍。 */
	float prev_rate_err_yaw_;

	/** 重力常数（m/s^2）。 */
	float k_gravity_;

	/** 外环 ny 通道比例增益。 */
	float k_acc_y_p_;
	/** 外环 ny 通道积分增益。 */
	float k_acc_y_i_;
	/** 外环 nz 通道比例增益。 */
	float k_acc_z_p_;
	/** 外环 nz 通道积分增益。 */
	float k_acc_z_i_;

	/** 内环 roll 通道比例增益。 */
	float k_rate_roll_p_;
	/** 内环 pitch 通道比例增益。 */
	float k_rate_pitch_p_;
	/** 内环 yaw 通道比例增益。 */
	float k_rate_yaw_p_;

	/** 内环 roll 通道微分增益。 */
	float k_rate_roll_d_;
	/** 内环 pitch 通道微分增益。 */
	float k_rate_pitch_d_;
	/** 内环 yaw 通道微分增益。 */
	float k_rate_yaw_d_;

	/** 外环 ny 积分限幅（抗饱和）。 */
	float i_limit_ny_;
	/** 外环 nz 积分限幅（抗饱和）。 */
	float i_limit_nz_;

	/** 外环输出到 pitch 角速度参考的限幅。 */
	float ref_rate_limit_pitch_;
	/** 外环输出到 yaw 角速度参考的限幅。 */
	float ref_rate_limit_yaw_;

	/** 控制器 R/P/Y 输出限幅。 */
	float cmd_limit_deg_;
	/** 舵面最终机械限幅。 */
	float fin_limit_deg_;
};

