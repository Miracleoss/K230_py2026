#ifndef GUIDANCE_TYPES_H
#define GUIDANCE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 制导输入数据结构
 * 
 * 包含视觉数据和姿态数据，每次更新时提供
 */
typedef struct {
    // 视觉数据
    float pixel_dx;      ///< x方向像素差（目标x - 图像中心x）
    float pixel_dy;      ///< y方向像素差（目标y - 图像中心y）
    float pixel_area;    ///< 目标像素面积
    
    // 姿态数据（四元数）
    float qw;           ///< 四元数w分量
    float qx;           ///< 四元数x分量
    float qy;           ///< 四元数y分量
    float qz;           ///< 四元数z分量
    
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
    float los_rate;      ///< 视线角速度 (rad/s)
    float est_distance;  ///< 估计距离 (m)
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
    
    // 距离估计参数
    float min_distance;  ///< 最小距离 (m)
    float max_distance;  ///< 最大距离 (m)
    float min_area;      ///< 最小像素面积
    float max_area;      ///< 最大像素面积
    
    // 相机参数
    float camera_matrix[9];  ///< 相机内参矩阵
    
    // 目标实际尺寸（用于距离估计）
    float target_real_size; ///< 目标实际尺寸 (mm)
    
    // 速度夹角（固定值或可更新）
    float velocity_angle; ///< 速度与基准面夹角 (rad)
    
    // 滤波器参数
    float filter_alpha;  ///< 低通滤波器系数 (0-1)
} GuidanceConfig;

/**
 * @brief 四元数结构
 */
typedef struct {
    float w, x, y, z;
} Quaternion;

/**
 * @brief 欧拉角结构（Z-Y-X顺序：yaw->pitch->roll）
 */
typedef struct {
    float yaw;    ///< 偏航角 (绕Z轴)
    float pitch;  ///< 俯仰角 (绕Y轴)
    float roll;   ///< 滚转角 (绕X轴)
} EulerAngles;

/**
 * @brief 系统状态结构（内部使用）
 */
typedef struct {
    // 基准四元数（发射瞬间）
    Quaternion reference_quat;
    
    // 历史数据
    float prev_los_angle;
    uint32_t prev_timestamp;
    
    // 当前估计距离
    float current_distance;
    
    // 系统状态
    bool initialized;
    bool reference_set;
} GuidanceState;

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_TYPES_H */
