#ifndef DART_GUIDANCE_H
#define DART_GUIDANCE_H

#include "guidance_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化制导系统
 * 
 * @param config 系统配置参数
 * @param ref_qw 基准四元数w分量
 * @param ref_qx 基准四元数x分量
 * @param ref_qy 基准四元数y分量
 * @param ref_qz 基准四元数z分量
 * 
 * @note 在系统启动时调用，发射前调用
 */
void guidance_init(GuidanceConfig config, 
                   float ref_qw, float ref_qx, float ref_qy, float ref_qz);

/**
 * @brief 更新基准四元数（发射瞬间调用）
 * 
 * @param qw 基准四元数w分量
 * @param qx 基准四元数x分量
 * @param qy 基准四元数y分量
 * @param qz 基准四元数z分量
 * 
 * @note 在检测到发射时调用，记录发射瞬间的姿态作为基准
 */
void guidance_set_reference_quaternion(float qw, float qx, float qy, float qz);

/**
 * @brief 更新视觉数据
 * 
 * @param dx x方向像素差（目标x - 图像中心x）
 * @param dy y方向像素差（目标y - 图像中心y）
 * @param area 目标像素面积
 * @param timestamp 时间戳（毫秒）
 * 
 * @note 每帧图像处理完成后调用
 */
void guidance_update_vision(float dx, float dy, float area, uint32_t timestamp);

/**
 * @brief 更新姿态数据
 * 
 * @param qw 当前四元数w分量
 * @param qx 当前四元数x分量
 * @param qy 当前四元数y分量
 * @param qz 当前四元数z分量
 * @param timestamp 时间戳（毫秒）
 * 
 * @note IMU数据更新时调用
 */
void guidance_update_attitude(float qw, float qx, float qy, float qz, uint32_t timestamp);

/**
 * @brief 计算制导指令（主函数）
 * 
 * @return GuidanceOutput 制导输出
 * 
 * @note 在控制周期（如100Hz）调用，返回俯仰和偏航方向的加速度指令
 */
GuidanceOutput guidance_calculate(void);

/**
 * @brief 设置估计距离（可选）
 * 
 * @param distance 估计距离（米）
 * 
 * @note 当有额外距离信息（如高度传感器）时调用
 */
void guidance_set_distance(float distance);

/**
 * @brief 设置速度夹角
 * 
 * @param angle 速度与基准平面夹角（弧度）
 * 
 * @note 用于修正视线角计算
 */
void guidance_set_velocity_angle(float angle);

/**
 * @brief 重置系统状态
 * 
 * @note 在重新发射前调用，清除历史数据
 */
void guidance_reset(void);

/**
 * @brief 获取当前系统状态
 * 
 * @return GuidanceOutput 当前制导状态
 * 
 * @note 用于调试和监控
 */
GuidanceOutput guidance_get_status(void);

/**
 * @brief 检查系统是否已初始化
 * 
 * @return true 系统已初始化
 * @return false 系统未初始化
 */
bool guidance_is_initialized(void);

/**
 * @brief 检查基准四元数是否已设置
 * 
 * @return true 基准已设置
 * @return false 基准未设置
 */
bool guidance_is_reference_set(void);

#ifdef __cplusplus
}
#endif

#endif /* DART_GUIDANCE_H */
