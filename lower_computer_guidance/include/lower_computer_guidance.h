#ifndef LOWER_COMPUTER_GUIDANCE_H
#define LOWER_COMPUTER_GUIDANCE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 制导输出结构
 */
typedef struct {
    float accel_pitch;   ///< 俯仰方向加速度 (m/s²)
    float accel_yaw;     ///< 偏航方向加速度 (m/s²)
    bool valid;          ///< 输出是否有效
} LowerComputerGuidanceOutput;

/**
 * @brief 系统配置结构
 */
typedef struct {
    // 制导参数
    float nav_ratio;     ///< 导航比 N' (建议4.0)
    
    // 相机内参矩阵（3x3，行主序）
    // [fx,  0, cx]
    // [ 0, fy, cy]
    // [ 0,  0,  1]
    float camera_matrix[9];  ///< 相机内参矩阵
    
    // 滤波器参数
    float filter_alpha;  ///< 低通滤波器系数 (0-1)

    // 加速度限制
    float max_accel_yaw;    ///< 偏航最大加速度 (m/s²)
    float max_accel_pitch;  ///< 俯仰最大加速度 (m/s²)
    
    // 角速度限制（用于异常检测）
    float max_los_rate;     ///< 最大视线角速度 (rad/s)
} LowerComputerGuidanceConfig;

/**
 * @brief 下位机制导律类
 * 
 * 核心算法：视线角速度 = IMU角速度 + 视觉目标相对角速度
 * 
 * 不再使用四元数+旋转矩阵变换，而是直接通过：
 * 1. IMU陀螺仪获得弹体自身角速度
 * 2. 视觉像素偏移计算目标相对弹体的角速度
 * 3. 两者合成得到惯性系下的视线角速度
 */
class LowerComputerGuidance
{
public:
    LowerComputerGuidance() = default;

    /**
     * @brief 初始化制导系统
     * 
     * @param config 系统配置参数
     */
    void guidance_init(const LowerComputerGuidanceConfig& config);

    /**
     * @brief 更新输入数据
     * 
     * @param dx x方向像素差（目标x - 光心cx）
     * @param dy y方向像素差（目标y - 光心cy）
     * @param gyro_x 弹体系x轴角速度（滚转, rad/s）
     * @param gyro_y 弹体系y轴角速度（俯仰, rad/s）
     * @param gyro_z 弹体系z轴角速度（偏航, rad/s）
     * @param target_velocity 弹目接近速度 (m/s)
     * @param timestamp 时间戳（毫秒）
     * 
     * @note 按照相机的帧率的更新来调用这个函数，而不是用imu数据更新，保证数据的时间戳一致
     */
    void guidance_update(float dx, float dy,
                         float gyro_x, float gyro_y, float gyro_z,
                         float target_velocity,
                         uint32_t timestamp);

    /**
     * @brief 计算制导指令
     * 
     * @return LowerComputerGuidanceOutput 制导输出
     */
    LowerComputerGuidanceOutput guidance_calculate(void);

    /**
     * @brief 设置发射标志
     */
    void fire(void);

    /**
     * @brief 重置系统状态
     */
    void guidance_reset(void);

private:
    // 配置参数
    LowerComputerGuidanceConfig g_config;

    // 输入数据
    float g_pixel_dx = 0.0f;
    float g_pixel_dy = 0.0f;
    float g_gyro_x = 0.0f;
    float g_gyro_y = 0.0f;
    float g_gyro_z = 0.0f;
    float g_target_velocity = 0.0f;
    uint32_t g_timestamp = 0;

    // 状态变量
    uint32_t pre_timestamp = 0;         ///< 上一次时间戳
    float prev_los_rate_yaw = 0.0f;     ///< 上一次偏航视线角速度（用于滤波）
    float prev_los_rate_pitch = 0.0f;   ///< 上一次俯仰视线角速度（用于滤波）
    float prev_pixel_dx = 0.0f;         ///< 上一次x方向像素差
    float prev_pixel_dy = 0.0f;         ///< 上一次y方向像素差

    // 标志位
    bool is_fire = false;
    bool initialized = false;

    // 内部工具函数
    float clamp_float(float value, float min, float max);
    float low_pass_filter(float input, float prev, float alpha);
};

#endif /* LOWER_COMPUTER_GUIDANCE_H */
