#include "lower_computer_guidance.h"
#include <math.h>
#include <float.h>

// 默认配置
static const LowerComputerGuidanceConfig DEFAULT_CONFIG = {
    .nav_ratio = 4.0f,           // 导航比 4.0
    .camera_matrix = {500.0f, 0.0f, 320.0f, 0.0f, 500.0f, 240.0f, 0.0f, 0.0f, 1.0f}, // fx=500, fy=500, cx=320, cy=240
    .filter_alpha = 0.3f,        // 滤波器系数 0.3
    .max_accel_yaw = 100.0f,     // 偏航最大加速度 100 m/s²
    .max_accel_pitch = 100.0f,   // 俯仰最大加速度 100 m/s²
    .max_los_rate = 10.0f        // 最大视线角速度 10 rad/s
};

void LowerComputerGuidance::guidance_init(const LowerComputerGuidanceConfig& config) {
    this->g_config = config;
    this->initialized = true;
    this->is_fire = false;
    this->pre_timestamp = 0;
    this->prev_los_rate_yaw = 0.0f;
    this->prev_los_rate_pitch = 0.0f;
    this->prev_pixel_dx = 0.0f;
    this->prev_pixel_dy = 0.0f;

    // 初始化输入
    this->g_pixel_dx = 0.0f;
    this->g_pixel_dy = 0.0f;
    this->g_gyro_x = 0.0f;
    this->g_gyro_y = 0.0f;
    this->g_gyro_z = 0.0f;
    this->g_target_velocity = 0.0f;
    this->g_timestamp = 0;
}

void LowerComputerGuidance::fire() {
    this->is_fire = true;
}

void LowerComputerGuidance::guidance_update(float dx, float dy,
                                            float gyro_x, float gyro_y, float gyro_z,
                                            float target_velocity,
                                            uint32_t timestamp) {
    // 更新输入数据
    this->g_pixel_dx = dx;
    this->g_pixel_dy = dy;
    this->g_gyro_x = gyro_x;
    this->g_gyro_y = gyro_y;
    this->g_gyro_z = gyro_z;
    this->g_target_velocity = target_velocity;
    this->g_timestamp = timestamp;
}

LowerComputerGuidanceOutput LowerComputerGuidance::guidance_calculate(void) {
    // 检查发射标志
    if (!this->is_fire) {
        LowerComputerGuidanceOutput invalid_output = {0};
        invalid_output.valid = false;
        return invalid_output;
    }

    // 检查初始化状态
    if (!this->initialized) {
        LowerComputerGuidanceOutput invalid_output = {0};
        invalid_output.valid = false;
        return invalid_output;
    }

    // 检查时间戳有效性
    if (this->g_timestamp == 0) {
        LowerComputerGuidanceOutput invalid_output = {0};
        invalid_output.valid = false;
        return invalid_output;
    }

    // 从相机内参矩阵提取参数
    float fx = this->g_config.camera_matrix[0];  // [0][0]
    float fy = this->g_config.camera_matrix[4];  // [1][1]
    float cx = this->g_config.camera_matrix[2];  // [0][2]
    float cy = this->g_config.camera_matrix[5];  // [1][2]


    // 第一次调用：记录时间戳和像素数据，返回零输出
    if (this->pre_timestamp == 0) {
        this->pre_timestamp = this->g_timestamp;
        this->prev_pixel_dx = this->g_pixel_dx;
        this->prev_pixel_dy = this->g_pixel_dy;

        LowerComputerGuidanceOutput zero_output = {0};
        zero_output.valid = true;
        return zero_output;
    }

    // 计算时间差（毫秒转换为秒）
    float dt = (float)(this->g_timestamp - this->pre_timestamp) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) {
        // 时间差异常，重置并返回无效
        LowerComputerGuidanceOutput invalid_output = {0};
        invalid_output.valid = false;
        return invalid_output;
    }

    // ========== 核心算法：视线角速度合成 ==========

    // 1. 视觉计算：像素速度 → 目标相对弹体的角速度
    //    像素速度 (pixel/s)
    float pixel_vx = (this->g_pixel_dx - this->prev_pixel_dx) / dt;
    float pixel_vy = (this->g_pixel_dy - this->prev_pixel_dy) / dt;

    // 1.1 弹体滚转补偿：利用gyro_x修正弹体自旋导致的像素角速度观测误差
    //    弹体滚转会导致相机坐标系旋转，使像素速度的偏航/俯仰分量被耦合
    //    通过反向旋转像素速度向量来补偿滚转影响
    float roll_angle = this->g_gyro_x * dt;  // 两次观测间的滚转角度增量
    float cos_roll = cosf(roll_angle);
    float sin_roll = sinf(roll_angle);
    float pixel_vx_compensated = pixel_vx * cos_roll + pixel_vy * sin_roll;
    float pixel_vy_compensated = -pixel_vx * sin_roll + pixel_vy * cos_roll;

    //    利用相机内参将补偿后的像素速度转换为角速度 (rad/s)
    //    小角度近似：tan(θ) ≈ θ，θ = pixel / focal_length
    //    dx, dy 已经是相对于光心(cx, cy)的偏移量，直接除以焦距即可
    float target_rel_yaw_rate = pixel_vx_compensated / fx;
    float target_rel_pitch_rate = pixel_vy_compensated / fy;

    // 2. IMU获取弹体角速度（直接使用输入的陀螺仪数据）
    //    gyro_z: 偏航角速度, gyro_y: 俯仰角速度
    float body_yaw_rate = this->g_gyro_z;
    float body_pitch_rate = this->g_gyro_y;

    // 3. 合成视线角速度（核心创新）
    //    视线角速度 = 弹体角速度 + 目标相对弹体的角速度
    //    物理意义：弹体自身在转，目标也在相对弹体运动，两者叠加
    //    就是目标在惯性系中的视线角速度
    float los_rate_yaw = body_yaw_rate + target_rel_yaw_rate;
    float los_rate_pitch = body_pitch_rate + target_rel_pitch_rate;

    // 4. 异常检测：限制视线角速度范围
    if (fabsf(los_rate_yaw) > this->g_config.max_los_rate ||
        fabsf(los_rate_pitch) > this->g_config.max_los_rate) {
        // 角速度异常，可能为噪声，使用上一次的值
        los_rate_yaw = this->prev_los_rate_yaw;
        los_rate_pitch = this->prev_los_rate_pitch;
    }

    // 5. 应用低通滤波器平滑视线角速度
    los_rate_yaw = low_pass_filter(los_rate_yaw, this->prev_los_rate_yaw, this->g_config.filter_alpha);
    los_rate_pitch = low_pass_filter(los_rate_pitch, this->prev_los_rate_pitch, this->g_config.filter_alpha);

    // 6. 比例导引法计算加速度指令
    //    n = N' * Vc * λ_dot
    float n_yaw = this->g_config.nav_ratio * this->g_target_velocity * los_rate_yaw;
    float n_pitch = this->g_config.nav_ratio * this->g_target_velocity * los_rate_pitch;

    // 7. 限制加速度范围
    n_yaw = clamp_float(n_yaw, -this->g_config.max_accel_yaw, this->g_config.max_accel_yaw);
    n_pitch = clamp_float(n_pitch, -this->g_config.max_accel_pitch, this->g_config.max_accel_pitch);

    // 更新历史数据
    this->prev_los_rate_yaw = los_rate_yaw;
    this->prev_los_rate_pitch = los_rate_pitch;
    this->prev_pixel_dx = this->g_pixel_dx;
    this->prev_pixel_dy = this->g_pixel_dy;
    this->pre_timestamp = this->g_timestamp;

    // 准备输出
    LowerComputerGuidanceOutput output;
    output.accel_pitch = n_pitch;
    output.accel_yaw = n_yaw;
    output.valid = true;

    return output;
}

void LowerComputerGuidance::guidance_reset(void) {
    this->initialized = false;
    this->is_fire = false;
    this->pre_timestamp = 0;
    this->prev_los_rate_yaw = 0.0f;
    this->prev_los_rate_pitch = 0.0f;
    this->prev_pixel_dx = 0.0f;
    this->prev_pixel_dy = 0.0f;

    // 重置输入
    this->g_pixel_dx = 0.0f;
    this->g_pixel_dy = 0.0f;
    this->g_gyro_x = 0.0f;
    this->g_gyro_y = 0.0f;
    this->g_gyro_z = 0.0f;
    this->g_target_velocity = 0.0f;
    this->g_timestamp = 0;
}

// ========== 内部工具函数 ==========

float LowerComputerGuidance::clamp_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float LowerComputerGuidance::low_pass_filter(float input, float prev, float alpha) {
    alpha = clamp_float(alpha, 0.0f, 1.0f);
    return prev + alpha * (input - prev);
}
