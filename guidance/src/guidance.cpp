#include "guidance.h"
#include <math.h>
#include <float.h>

void Guidance::guidance_init(const GuidanceConfig& config) {
    this->g_config = config;
    this->initialized = true;
    this->is_fire = false;
    this->pre_timestamp = 0;
    this->prev_los_rate_yaw = 0.0f;
    this->prev_los_rate_pitch = 0.0f;
    this->prev_pixel_dx = 0.0f;
    this->prev_pixel_dy = 0.0f;
    this->prev_closing_velocity = 0.0f;
    this->prev_filtered_gyro_x = 0.0f;
    this->lost_frame_count = 0;
}

void Guidance::fire() {
    this->is_fire = true;
}

float Guidance::compute_adaptive_alpha(float target_area) {
    // 目标丢失或阈值无效时使用强滤波（低alpha），避免噪声导致指令抖动
    if (target_area <= 0.0f || this->g_config.area_threshold <= 0.0f) {
        return this->g_config.filter_alpha_min;
    }

    if (target_area >= this->g_config.area_threshold) {
        return this->g_config.filter_alpha_max;
    }

    float ratio = target_area / this->g_config.area_threshold;
    float alpha = this->g_config.filter_alpha_min +
                  (this->g_config.filter_alpha_max - this->g_config.filter_alpha_min) * ratio;
    return clamp_float(alpha, this->g_config.filter_alpha_min, this->g_config.filter_alpha_max);
}

GuidanceOutput Guidance::guidance_process(float dx, float dy,
                                           float gyro_x, float gyro_y, float gyro_z,
                                           float target_area,
                                           float closing_velocity,
                                           uint32_t timestamp) {
    // 安全检查
    if (!this->is_fire || !this->initialized || timestamp == 0) {
        GuidanceOutput invalid = {0};
        invalid.valid = false;
        return invalid;
    }

    // NaN/Inf 输入保护：传感器异常时保持上一帧有效指令
    if (isnanf(dx) || isnanf(dy) || isinff(dx) || isinff(dy) ||
        isnanf(gyro_x) || isnanf(gyro_y) || isnanf(gyro_z) ||
        isinff(gyro_x) || isinff(gyro_y) || isinff(gyro_z) ||
        isnanf(closing_velocity) || isinff(closing_velocity)) {
        GuidanceOutput hold;
        hold.accel_pitch = this->g_config.nav_ratio * this->prev_closing_velocity * this->prev_los_rate_pitch;
        hold.accel_yaw   = this->g_config.nav_ratio * this->prev_closing_velocity * this->prev_los_rate_yaw;
        hold.accel_pitch = clamp_float(hold.accel_pitch, -this->g_config.max_accel_pitch, this->g_config.max_accel_pitch);
        hold.accel_yaw   = clamp_float(hold.accel_yaw,   -this->g_config.max_accel_yaw,   this->g_config.max_accel_yaw);
        hold.valid = true;
        return hold;
    }

    float fx = this->g_config.camera_matrix[0];
    float fy = this->g_config.camera_matrix[4];

    // 第一帧：记录数据建立基线，返回零指令
    if (this->pre_timestamp == 0) {
        this->pre_timestamp = timestamp;
        this->prev_pixel_dx = dx;
        this->prev_pixel_dy = dy;
        this->prev_closing_velocity = closing_velocity;

        GuidanceOutput zero = {0};
        zero.valid = true;
        return zero;
    }

    // 时间差（毫秒→秒）
    float dt = (float)(timestamp - this->pre_timestamp) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) {
        GuidanceOutput invalid = {0};
        invalid.valid = false;
        return invalid;
    }

    // ========== 核心算法：视线角速度合成 ==========

    // 1. gyro_x 预滤波（抑制滚转速率噪声，避免滚转补偿引入高频抖动）
    //    使用固定 alpha=0.5 对 gyro_x 做一阶低通
    float filtered_gyro_x = this->prev_filtered_gyro_x +
                            0.5f * (gyro_x - this->prev_filtered_gyro_x);

    // 2. 计算原始像素速度
    float raw_pixel_vx = (dx - this->prev_pixel_dx) / dt;
    float raw_pixel_vy = (dy - this->prev_pixel_dy) / dt;

    // 3. 滚转补偿
    //    弹体自旋导致图像绕光心旋转，诱导像素运动为:
    //      vx_roll = -gyro_x * dy
    //      vy_roll =  gyro_x * dx
    //    从原始像素速度中减掉滚转诱导量得到真实目标运动
    float pixel_vx = raw_pixel_vx + filtered_gyro_x * dy;
    float pixel_vy = raw_pixel_vy - filtered_gyro_x * dx;

    float los_rate_yaw;
    float los_rate_pitch;

    // 4. 像素跳变异常检测
    float pixel_jump = sqrtf(
        (dx - this->prev_pixel_dx) * (dx - this->prev_pixel_dx) +
        (dy - this->prev_pixel_dy) * (dy - this->prev_pixel_dy)
    );

    if (pixel_jump > this->g_config.max_pixel_jump) {
        // 跟丢重捕：帧计数器累加，超时后LOS速率衰减归零
        this->lost_frame_count++;
        if (this->lost_frame_count > this->g_config.max_lost_frames) {
            los_rate_yaw = 0.0f;
            los_rate_pitch = 0.0f;
        } else {
            float decay = 1.0f - (float)this->lost_frame_count / (float)this->g_config.max_lost_frames;
            los_rate_yaw = this->prev_los_rate_yaw * decay;
            los_rate_pitch = this->prev_los_rate_pitch * decay;
        }
    } else {
        this->lost_frame_count = 0;

        // 5. 像素速度→目标相对角速度
        //    针孔模型: x=f·tan(θ) → dx/dt=f·sec²(θ)·dθ/dt
        //    解得: dθ/dt = (dx/dt)·cos²(θ)/f
        //    cos² 修正补偿大视场角下的 sec² 因子，FOV边缘(~30°)误差约33%
        float theta_x = atanf(dx / fx);
        float theta_y = atanf(dy / fy);
        float cos2_x = cosf(theta_x) * cosf(theta_x);
        float cos2_y = cosf(theta_y) * cosf(theta_y);
        float target_rel_yaw_rate = atanf(pixel_vx / fx) * cos2_x;
        float target_rel_pitch_rate = atanf(pixel_vy / fy) * cos2_y;

        // 6. 合成视线角速度
        //    视线角速度 = 弹体角速度 + 目标相对弹体的角速度
        los_rate_yaw = gyro_z + target_rel_yaw_rate;
        los_rate_pitch = gyro_y + target_rel_pitch_rate;

        // 7. 视线角速度异常检测
        if (fabsf(los_rate_yaw) > this->g_config.max_los_rate ||
            fabsf(los_rate_pitch) > this->g_config.max_los_rate) {
            los_rate_yaw = this->prev_los_rate_yaw;
            los_rate_pitch = this->prev_los_rate_pitch;
        }
    }

    // 8. 自适应低通滤波
    float alpha = compute_adaptive_alpha(target_area);
    los_rate_yaw = low_pass_filter(los_rate_yaw, this->prev_los_rate_yaw, alpha);
    los_rate_pitch = low_pass_filter(los_rate_pitch, this->prev_los_rate_pitch, alpha);

    // 9. 比例导引法 n = N' * Vc * λ̇
    float n_yaw = this->g_config.nav_ratio * closing_velocity * los_rate_yaw;
    float n_pitch = this->g_config.nav_ratio * closing_velocity * los_rate_pitch;

    n_yaw = clamp_float(n_yaw, -this->g_config.max_accel_yaw, this->g_config.max_accel_yaw);
    n_pitch = clamp_float(n_pitch, -this->g_config.max_accel_pitch, this->g_config.max_accel_pitch);

    // 10. 重力补偿（可选前馈）
    //     叠加后不受限幅约束：限幅针对制导律输出，重力是物理力
    if (this->g_config.enable_gravity_comp) {
        float pitch_angle = atanf(dy / fy);
        n_pitch += this->g_config.gravity_mag * cosf(pitch_angle);
    }

    // 更新历史数据
    this->prev_los_rate_yaw = los_rate_yaw;
    this->prev_los_rate_pitch = los_rate_pitch;
    this->prev_pixel_dx = dx;
    this->prev_pixel_dy = dy;
    this->prev_closing_velocity = closing_velocity;
    this->prev_filtered_gyro_x = filtered_gyro_x;
    this->pre_timestamp = timestamp;

    GuidanceOutput output;
    output.accel_pitch = n_pitch;
    output.accel_yaw = n_yaw;
    output.valid = true;

    return output;
}

void Guidance::guidance_reset(void) {
    this->initialized = false;
    this->is_fire = false;
    this->pre_timestamp = 0;
    this->prev_los_rate_yaw = 0.0f;
    this->prev_los_rate_pitch = 0.0f;
    this->prev_pixel_dx = 0.0f;
    this->prev_pixel_dy = 0.0f;
    this->prev_closing_velocity = 0.0f;
    this->prev_filtered_gyro_x = 0.0f;
    this->lost_frame_count = 0;
}

// ========== 内部工具函数 ==========

float Guidance::clamp_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float Guidance::low_pass_filter(float input, float prev, float alpha) {
    alpha = clamp_float(alpha, 0.0f, 1.0f);
    return prev + alpha * (input - prev);
}
