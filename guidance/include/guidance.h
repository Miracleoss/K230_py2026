#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 制导输出结构
 */
typedef struct {
    float accel_pitch;   ///< 俯仰方向加速度 (m/s²)
    float accel_yaw;     ///< 偏航方向加速度 (m/s²)
    bool valid;          ///< 输出是否有效
} GuidanceOutput;

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
    float camera_matrix[9];

    // 自适应滤波参数
    float filter_alpha_min;  ///< 最小alpha（远距离/小目标，强滤波）
    float filter_alpha_max;  ///< 最大alpha（近距离/大目标，快速响应）
    float area_threshold;    ///< 目标面积阈值，面积 >= 此值时使用 alpha_max

    // 加速度限制
    float max_accel_yaw;    ///< 偏航最大加速度 (m/s²)
    float max_accel_pitch;  ///< 俯仰最大加速度 (m/s²)

    // 异常检测参数
    float max_los_rate;       ///< 最大视线角速度 (rad/s)
    float max_pixel_jump;     ///< 最大帧间像素跳变，超过视为跟丢重捕
    int   max_lost_frames;    ///< 目标丢失容忍帧数，超时后LOS速率归零 (建议10帧@60Hz)

    // 重力补偿
    bool  enable_gravity_comp; ///< 是否启用重力补偿前馈 (默认关闭)
    float gravity_mag;         ///< 重力加速度大小 (m/s², 默认9.81)
} GuidanceConfig;

/**
 * @brief 制导律类
 *
 * 核心算法：视线角速度 = IMU角速度 + 视觉目标相对角速度
 * 通过比例导引法计算加速度指令。
 *
 * IMU陀螺仪获得弹体自身角速度，视觉像素偏移计算目标相对弹体的角速度，
 * 两者合成得到惯性系下的视线角速度。
 *
 * 调用方式（单帧处理，帧率60Hz）：
 * @code
 *   Guidance g;
 *   g.guidance_init(config);
 *   g.fire();                    // 发射后置位
 *   // 每帧调用一次
 *   auto out = g.guidance_process(dx, dy, gx, gy, gz, area, vel, ts);
 * @endcode
 */
class Guidance
{
public:
    Guidance() = default;

    /**
     * @brief 初始化制导系统
     * @param config 系统配置参数
     */
    void guidance_init(const GuidanceConfig& config);

    /**
     * @brief 设置发射标志
     * @note 发射后调用，之后 guidance_process 才会输出有效指令
     */
    void fire(void);

    /**
     * @brief 处理一帧制导数据
     *
     * @param dx x方向像素差（目标x - 光心cx）
     * @param dy y方向像素差（目标y - 光心cy）
     * @param gyro_x 弹体系x轴角速度（滚转, rad/s）
     * @param gyro_y 弹体系y轴角速度（俯仰, rad/s）
     * @param gyro_z 弹体系z轴角速度（偏航, rad/s）
     * @param target_area 目标像素面积（用于自适应滤波距离判断）
     * @param closing_velocity 弹目接近速度 (m/s)
     * @param timestamp 时间戳（毫秒）
     *
     * @return GuidanceOutput 制导输出
     *   - 第一帧返回 zero_output.valid = true（需要一帧建立速度基线）
     *   - 后续返回有效加速度指令
     *
     * @note 按相机帧率（如60Hz）每帧调用一次
     */
    GuidanceOutput guidance_process(float dx, float dy,
                                    float gyro_x, float gyro_y, float gyro_z,
                                    float target_area,
                                    float closing_velocity,
                                    uint32_t timestamp);

    /**
     * @brief 重置系统状态
     * @note 重新发射前调用
     */
    void guidance_reset(void);

private:
    // 配置参数
    GuidanceConfig g_config;

    // 上一帧数据（用于帧间差分）
    uint32_t pre_timestamp = 0;
    float prev_los_rate_yaw = 0.0f;
    float prev_los_rate_pitch = 0.0f;
    float prev_pixel_dx = 0.0f;
    float prev_pixel_dy = 0.0f;
    float prev_closing_velocity = 0.0f;  // 用于输入异常时保持指令

    // gyro_x 预滤波
    float prev_filtered_gyro_x = 0.0f;

    // 目标丢失帧计数
    int lost_frame_count = 0;

    // 标志位
    bool is_fire = false;
    bool initialized = false;

    // 内部工具函数
    float clamp_float(float value, float min, float max);
    float low_pass_filter(float input, float prev, float alpha);
    float compute_adaptive_alpha(float target_area);
};

#endif /* GUIDANCE_H */
