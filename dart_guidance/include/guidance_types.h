#ifndef GUIDANCE_TYPES_H
#define GUIDANCE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 四元数结构
 */
typedef struct {
    float w, x, y, z;
} Quaternion;

/**
 * @brief 制导输入数据结构
 * 
 * 包含视觉数据和姿态数据，每次更新时提供
 */
typedef struct {
    // 视觉数据
    float pixel_dx;      ///< x方向像素差（目标x - 图像中心x）
    float pixel_dy;      ///< y方向像素差（目标y - 图像中心y）
    
    // 姿态数据（四元数）
    Quaternion q;

    // 弹目接近速度
    float target_velocity; ///< 目标接近速度 (m/s)
    
    // 时间戳（毫秒）
    uint32_t timestamp;
} GuidanceInput;

/**
 * @brief 制导输出数据结构
 * 
 * 包含解耦的加速度指令和状态信息
 */
typedef struct {
    float accel_pitch;   ///< 俯仰方向加速度 (m/s²)
    float accel_yaw;     ///< 偏航方向加速度 (m/s²)
    bool valid;          ///< 输出是否有效
} GuidanceOutput;

/**
 * @brief 系统配置结构
 * 
 * 包含制导算法所有可配置参数
 */
typedef struct {
    // 制导参数
    float nav_ratio;     ///< 导航比 N' (建议4.0)
    
    // 相机参数
    float camera_matrix[9];  ///< 相机内参矩阵
    
    // 滤波器参数
    float filter_alpha;  ///< 低通滤波器系数 (0-1)
} GuidanceConfig;



/**
 * @brief 欧拉角结构（Z-Y-X顺序：yaw->pitch->roll）
 */
typedef struct {
    float yaw;    ///< 偏航角 (绕Z轴)
    float pitch;  ///< 俯仰角 (绕Y轴)
    float roll;   ///< 滚转角 (绕X轴)
} EulerAngles;

/**
 * @brief 三维坐标(x,y,z)
 */
typedef struct 
{
    float x;
    float y;
    float z;
}Cordation;


#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_TYPES_H */
