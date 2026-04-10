#include "guidance_math.h"
#include <math.h>
#include <float.h>

float clamp_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float lerp_float(float a, float b, float t) {
    t = clamp_float(t, 0.0f, 1.0f);
    return a + (b - a) * t;
}

float safe_atan2(float y, float x) {
    // 处理特殊情况
    if (fabsf(x) < FLT_EPSILON && fabsf(y) < FLT_EPSILON) {
        return 0.0f;
    }
    return atan2f(y, x);
}

float quaternion_norm(Quaternion q) {
    return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

Quaternion quaternion_normalize(Quaternion q) {
    float norm = quaternion_norm(q);
    if (norm > FLT_EPSILON) {
        Quaternion result = {
            .w = q.w / norm,
            .x = q.x / norm,
            .y = q.y / norm,
            .z = q.z / norm
        };
        return result;
    }
    // 如果模长为0，返回单位四元数
    return (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};
}

Quaternion quaternion_conjugate(Quaternion q) {
    return (Quaternion){q.w, -q.x, -q.y, -q.z};
}

Quaternion quaternion_multiply(Quaternion a, Quaternion b) {
    Quaternion result;
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return result;
}

Quaternion quaternion_relative(Quaternion current, Quaternion reference) {
    // 计算相对四元数：Q_rel = Q_curr × conj(Q_ref)
    Quaternion ref_conj = quaternion_conjugate(reference);
    return quaternion_multiply(current, ref_conj);
}

float low_pass_filter(float input, float prev, float alpha) {
    alpha = clamp_float(alpha, 0.0f, 1.0f);
    return prev + alpha * (input - prev);
}

float angle_difference(float angle1, float angle2) {
    float diff = angle1 - angle2;
    
    // 将角度差限制在[-π, π]范围内
    while (diff > M_PI) {
        diff -= 2.0f * M_PI;
    }
    while (diff < -M_PI) {
        diff += 2.0f * M_PI;
    }
    
    return diff;
}

/**
 * @brief 从四元数计算欧拉角（Z-Y-X顺序：yaw->pitch->roll）
 * 
 * 按照技术报告中的欧拉角更新顺序：
 * Z(yaw) -> Y(pitch) -> X(roll)
 */
EulerAngles quaternion_to_euler_zyx(Quaternion q) {
    EulerAngles angles;
    
    // 归一化四元数
    q = quaternion_normalize(q);
    
    // 计算欧拉角（Z-Y-X顺序）
    // 参考：https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    angles.roll = atan2f(sinr_cosp, cosr_cosp);
    
    // pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f) {
        // 使用90度如果超出范围
        angles.pitch = copysignf(M_PI / 2.0f, sinp);
    } else {
        angles.pitch = asinf(sinp);
    }
    
    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2f(siny_cosp, cosy_cosp);
    
    return angles;
}

/**
 * @brief 从像素坐标计算角度（使用相机焦距）
 * 
 * 使用相机参数将像素差转换为实际角度
 * 公式：θ = atan2(pixel, focal_length)
 */
float pixel_to_angle(float pixel, float focal_length) {
    if (fabsf(focal_length) < FLT_EPSILON) {
        return 0.0f;
    }
    return atan2f(pixel, focal_length);
}

/**
 * @brief 基于相机参数和面积估算距离
 * 
 * 使用相机焦距和目标实际尺寸估算距离
 * 公式：距离 = (实际尺寸 × 焦距) / 像素尺寸
 */
float estimate_distance_from_camera(float pixel_area, float real_size, float focal_length) {
    if (pixel_area < FLT_EPSILON) {
        return 0.0f;
    }
    
    // 假设目标近似为圆形，计算像素半径
    float pixel_radius = sqrtf(pixel_area / M_PI);
    
    // 使用相似三角形原理计算距离
    // 距离 = (实际半径 × 焦距) / 像素半径
    float real_radius = real_size / 2.0f; // 假设real_size是直径
    return (real_radius * focal_length) / pixel_radius;
}
