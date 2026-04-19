#ifndef GUIDANCE_MATH_H
#define GUIDANCE_MATH_H

#include "guidance_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// 数学常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

/**
 * @brief 限制值在指定范围内
 * 
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return float 限制后的值
 */
float clamp_float(float value, float min, float max);

/**
 * @brief 线性插值
 * 
 * @param a 起始值
 * @param b 结束值
 * @param t 插值系数 (0-1)
 * @return float 插值结果
 */
float lerp_float(float a, float b, float t);

/**
 * @brief 计算反正切（atan2），返回角度在[-π, π]之间
 * 
 * @param y y坐标
 * @param x x坐标
 * @return float 角度（弧度）
 */
float safe_atan2(float y, float x);

/**
 * @brief 计算四元数模长
 * 
 * @param q 四元数
 * @return float 模长
 */
float quaternion_norm(Quaternion q);

/**
 * @brief 四元数归一化
 * 
 * @param q 输入四元数
 * @return Quaternion 归一化后的四元数
 */
Quaternion quaternion_normalize(Quaternion q);

/**
 * @brief 四元数共轭
 * 
 * @param q 输入四元数
 * @return Quaternion 共轭四元数
 */
Quaternion quaternion_conjugate(Quaternion q);

/**
 * @brief 四元数乘法
 * 
 * @param a 第一个四元数
 * @param b 第二个四元数
 * @return Quaternion 乘积
 */
Quaternion quaternion_multiply(Quaternion a, Quaternion b);

/**
 * @brief 计算相对四元数（当前姿态相对于基准姿态）
 * 
 * @param current 当前四元数
 * @param reference 基准四元数
 * @return Quaternion 相对四元数
 */
Quaternion quaternion_relative(Quaternion current, Quaternion reference);

/**
 * @brief 一阶低通滤波器
 * 
 * @param input 输入值
 * @param prev 前一次输出值
 * @param alpha 滤波器系数 (0-1)
 * @return float 滤波后的值
 */
float low_pass_filter(float input, float prev, float alpha);

/**
 * @brief 计算角度差（考虑周期边界）
 * 
 * @param angle1 角度1（弧度）
 * @param angle2 角度2（弧度）
 * @return float 角度差（弧度，在[-π, π]之间）
 */
float angle_difference(float angle1, float angle2);

/**
 * @brief 从四元数计算欧拉角（Z-Y-X顺序：yaw->pitch->roll）
 * 
 * 按照技术报告中的欧拉角更新顺序：
 * Z(yaw) -> Y(pitch) -> X(roll)
 * 
 * @param q 输入四元数
 * @return EulerAngles 欧拉角
 */
EulerAngles quaternion_to_euler_zyx(Quaternion q);

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_MATH_H */
