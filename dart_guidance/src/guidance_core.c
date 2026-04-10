#include "guidance_math.h"
#include <math.h>
#include <float.h>

// 内部函数声明
static float estimate_distance_from_area(float pixel_area, float min_area, float max_area, 
                                        float min_dist, float max_dist);
static float calculate_line_of_sight_angle(float pixel_dx, float pixel_dy, 
                                          float focal_length, float velocity_angle);
static void generalized_pn_guidance(float los_rate, float distance, float nav_ratio,
                                   float* accel_pitch, float* accel_yaw);
static float estimate_distance_from_camera_params(float pixel_area, float real_size, 
                                                 float focal_length, float min_dist, float max_dist);


/**
 * @brief 基于相机参数和面积估算距离（新方法）
 * 
 * 使用相机焦距和目标实际尺寸估算距离
 * 公式：距离 = (实际尺寸 × 焦距) / 像素尺寸
 */
static float estimate_distance_from_camera_params(float pixel_area, float real_size, 
                                                 float focal_length, float min_dist, float max_dist) {
    if (pixel_area < FLT_EPSILON || fabsf(focal_length) < FLT_EPSILON) {
        return max_dist; // 返回最大距离作为默认值
    }
    
    // 使用相机参数计算距离
    float distance = estimate_distance_from_camera(pixel_area, real_size, focal_length);
    
    // 限制在合理范围内
    return clamp_float(distance, min_dist, max_dist);
}

/**
 * @brief 计算视线角（使用相机焦距计算实际角度）
 * 
 * 实现角 = 使用焦距计算的图像角度 - 速度与基准平面的夹角
 * 公式：θ_x = atan2(pixel_dx, focal_length), θ_y = atan2(pixel_dy, focal_length)
 */
static float calculate_line_of_sight_angle(float pixel_dx, float pixel_dy,
                                          float focal_length, float velocity_angle) {
    // 使用相机焦距计算实际角度
    float angle_x = pixel_to_angle(pixel_dx, focal_length);
    float angle_y = pixel_to_angle(pixel_dy, focal_length);
    
    // 计算合成角度（假设主要关注水平方向）
    // 这里可以根据需要调整，例如使用atan2(angle_y, angle_x)计算合成角度
    float image_angle = atan2f(angle_y, angle_x);
    
    // 修正：减去速度与基准平面的夹角
    // 注意：这里假设速度夹角是已知的固定值或可测量值
    float corrected_angle = image_angle - velocity_angle;
    
    // 将角度限制在[-π, π]范围内
    while (corrected_angle > M_PI) {
        corrected_angle -= 2.0f * M_PI;
    }
    while (corrected_angle < -M_PI) {
        corrected_angle += 2.0f * M_PI;
    }
    
    return corrected_angle;
}

/**
 * @brief 广义比例导引律计算
 * 
 * 公式：加速度 = N' × R × q_dot
 * 输出解耦的俯仰和偏航加速度
 */
static void generalized_pn_guidance(float los_rate, float distance, float nav_ratio,
                                   float* accel_pitch, float* accel_yaw) {
    // 基本比例导引公式
    float acceleration_magnitude = nav_ratio * distance * los_rate;
    
    // 解耦分配：假设视线角速度主要在偏航方向
    // 实际分配比例需要根据飞行力学调整
    // 这里使用简单的固定比例分配
    
    // 偏航方向：主要响应水平方向的视线变化
    *accel_yaw = acceleration_magnitude * 0.7f;  // 70%给偏航
    
    // 俯仰方向：响应垂直方向的视线变化
    // 注意：这里简化处理，实际可能需要根据高度差调整
    *accel_pitch = acceleration_magnitude * 0.3f; // 30%给俯仰
    
    // 限制输出范围（防止过大指令）
    float max_accel = 50.0f; // 最大加速度 50 m/s² (约5G)
    *accel_pitch = clamp_float(*accel_pitch, -max_accel, max_accel);
    *accel_yaw = clamp_float(*accel_yaw, -max_accel, max_accel);
}

/**
 * @brief 计算视线角速度
 * 
 * 使用历史数据计算角速度，应用低通滤波平滑
 */
static float calculate_line_of_sight_rate(float current_angle, float prev_angle,
                                         uint32_t current_time, uint32_t prev_time,
                                         float* filtered_rate, float filter_alpha) {
    // 计算时间差（秒）
    float dt = (current_time - prev_time) / 1000.0f; // 毫秒转秒
    
    // 避免除零和无效时间差
    if (dt < 0.001f) {
        return *filtered_rate; // 返回上一次滤波后的值
    }
    
    // 计算原始角速度
    float angle_diff = angle_difference(current_angle, prev_angle);
    float raw_rate = angle_diff / dt;
    
    // 应用低通滤波平滑角速度
    *filtered_rate = low_pass_filter(raw_rate, *filtered_rate, filter_alpha);
    
    return *filtered_rate;
}

/**
 * @brief 处理姿态数据，计算相对姿态和欧拉角
 * 
 * 使用基准四元数计算当前姿态相对于发射基准的变化
 * 返回欧拉角（Z-Y-X顺序）
 */
static EulerAngles process_attitude_data(float qw, float qx, float qy, float qz,
                                       Quaternion reference_quat) {
    // 构造当前四元数
    Quaternion current_quat = {qw, qx, qy, qz};
    current_quat = quaternion_normalize(current_quat);
    
    // 计算相对于基准的姿态
    Quaternion relative_quat = quaternion_relative(current_quat, reference_quat);
    
    // 转换为欧拉角（Z-Y-X顺序）
    return quaternion_to_euler_zyx(relative_quat);
}

/**
 * @brief 完整的制导计算流程（改进版）
 * 
 * 整合所有步骤：距离估计、视线角计算、角速度计算、比例导引
 * 使用相机参数和欧拉角更新顺序
 */
GuidanceOutput calculate_guidance_output(GuidanceInput input, GuidanceConfig config,
                                        GuidanceState* state, GuidanceOutput* prev_output) {
    GuidanceOutput output = {0};
    
    // 检查输入有效性
    if (!state->initialized || !state->reference_set) {
        output.valid = false;
        return output;
    }
    
    // 1. 估计距离（基于相机参数和面积）
    float estimated_distance;
    if (config.target_real_size > FLT_EPSILON) {
        // 使用相机参数方法（新方法）
        estimated_distance = estimate_distance_from_camera_params(
            input.pixel_area,
            config.target_real_size,
            config.focal_length,
            config.min_distance,
            config.max_distance
        );
    } else {
        // 使用面积线性映射方法（兼容模式）
        estimated_distance = estimate_distance_from_area(
            input.pixel_area,
            config.min_area, config.max_area,
            config.min_distance, config.max_distance
        );
    }
    
    // 更新状态中的距离估计
    state->current_distance = estimated_distance;
    
    // 2. 计算视线角（使用相机焦距计算实际角度）
    float los_angle = calculate_line_of_sight_angle(
        input.pixel_dx, input.pixel_dy,
        config.focal_length, config.velocity_angle
    );
    
    // 3. 计算视线角速度（需要历史数据）
    float los_rate = 0.0f;
    if (state->prev_timestamp > 0 && input.timestamp > state->prev_timestamp) {
        los_rate = calculate_line_of_sight_rate(
            los_angle, state->prev_los_angle,
            input.timestamp, state->prev_timestamp,
            &prev_output->los_rate, config.filter_alpha
        );
    }
    
    // 4. 应用广义比例导引律
    float accel_pitch = 0.0f, accel_yaw = 0.0f;
    generalized_pn_guidance(los_rate, estimated_distance, config.nav_ratio,
                           &accel_pitch, &accel_yaw);
    
    // 5. 处理姿态数据（计算欧拉角，用于可能的姿态修正）
    EulerAngles relative_angles = process_attitude_data(
        input.qw, input.qx, input.qy, input.qz,
        state->reference_quat
    );
    
    // 注意：这里可以根据相对欧拉角修正加速度指令
    // 例如，考虑滚转角对俯仰和偏航控制的影响
    // 当前版本简化处理，暂不使用姿态修正
    
    // 6. 更新历史数据
    state->prev_los_angle = los_angle;
    state->prev_timestamp = input.timestamp;
    
    // 7. 填充输出结构
    output.accel_pitch = accel_pitch;
    output.accel_yaw = accel_yaw;
    output.los_rate = los_rate;
    output.est_distance = estimated_distance;
    output.valid = true;
    
    return output;
}
