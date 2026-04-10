#include "dart_guidance.h"
#include "guidance_math.h"
#include "guidance_core.h"
#include <string.h>

// 全局状态变量（单例模式，适合嵌入式系统）
static GuidanceState g_state = {0};
static GuidanceConfig g_config = {0};
static GuidanceInput g_current_input = {0};
static GuidanceOutput g_current_output = {0};
static GuidanceOutput g_prev_output = {0};

// 默认配置
static const GuidanceConfig DEFAULT_CONFIG = {
    .nav_ratio = 4.0f,           // 导航比 4.0
    .min_distance = 5.0f,        // 最小距离 5m
    .max_distance = 25.0f,       // 最大距离 25m
    .min_area = 10.0f,           // 最小像素面积
    .max_area = 1000.0f,         // 最大像素面积
    .camera_matrix = {500.0f, 0.0f, 320.0f, 0.0f, 500.0f, 240.0f, 0.0f, 0.0f, 1.0f}, // 焦距 500像素
    .target_real_size = 55.0f,  // 目标实际尺寸 150mm（15cm直径）
    .velocity_angle = 0.1f,      // 速度夹角 0.1rad
    .filter_alpha = 0.3f         // 滤波器系数 0.3
};

void guidance_init(GuidanceConfig config, 
                   float ref_qw, float ref_qx, float ref_qy, float ref_qz) {
    // 复制配置
    memcpy(&g_config, &config, sizeof(GuidanceConfig));
    
    // 设置基准四元数
    g_state.reference_quat.w = ref_qw;
    g_state.reference_quat.x = ref_qx;
    g_state.reference_quat.y = ref_qy;
    g_state.reference_quat.z = ref_qz;
    g_state.reference_quat = quaternion_normalize(g_state.reference_quat);
    
    // 初始化状态
    g_state.prev_los_angle = 0.0f;
    g_state.prev_timestamp = 0;
    g_state.current_distance = g_config.max_distance; // 初始为最大距离
    g_state.initialized = true;
    g_state.reference_set = true;
    
    // 初始化输出
    memset(&g_current_output, 0, sizeof(GuidanceOutput));
    memset(&g_prev_output, 0, sizeof(GuidanceOutput));
    memset(&g_current_input, 0, sizeof(GuidanceInput));
    
    // 设置默认输出
    g_current_output.valid = false;
    g_prev_output.valid = false;
}

void guidance_set_reference_quaternion(float qw, float qx, float qy, float qz) {
    if (!g_state.initialized) {
        // 如果未初始化，使用默认配置初始化
        guidance_init(DEFAULT_CONFIG, qw, qx, qy, qz);
        return;
    }
    
    // 更新基准四元数
    g_state.reference_quat.w = qw;
    g_state.reference_quat.x = qx;
    g_state.reference_quat.y = qy;
    g_state.reference_quat.z = qz;
    g_state.reference_quat = quaternion_normalize(g_state.reference_quat);
    g_state.reference_set = true;
    
    // 重置历史数据（新的发射）
    g_state.prev_los_angle = 0.0f;
    g_state.prev_timestamp = 0;
    g_prev_output.los_rate = 0.0f;
}

void guidance_update_vision(float dx, float dy, float area, uint32_t timestamp) {
    if (!g_state.initialized) return;
    
    g_current_input.pixel_dx = dx;
    g_current_input.pixel_dy = dy;
    g_current_input.pixel_area = area;
    g_current_input.timestamp = timestamp;
    
    // 注意：这里只更新视觉部分，姿态数据需要单独更新
}

void guidance_update_attitude(float qw, float qx, float qy, float qz, uint32_t timestamp) {
    if (!g_state.initialized) return;
    
    g_current_input.qw = qw;
    g_current_input.qx = qx;
    g_current_input.qy = qy;
    g_current_input.qz = qz;
    
    // 如果时间戳未设置（视觉数据未更新），使用姿态时间戳
    if (g_current_input.timestamp == 0) {
        g_current_input.timestamp = timestamp;
    }
}

GuidanceOutput guidance_calculate(void) {
    if (!g_state.initialized || !g_state.reference_set) {
        GuidanceOutput invalid_output = {0};
        invalid_output.valid = false;
        return invalid_output;
    }
    
    // 检查输入数据是否有效
    if (g_current_input.timestamp == 0) {
        GuidanceOutput invalid_output = {0};
        invalid_output.valid = false;
        return invalid_output;
    }
    
    // 调用核心算法
    g_current_output = calculate_guidance_output(g_current_input, g_config, 
                                                &g_state, &g_prev_output);
    
    // 保存当前输出作为下一次的历史数据
    if (g_current_output.valid) {
        g_prev_output = g_current_output;
    }
    
    return g_current_output;
}

void guidance_set_distance(float distance) {
    if (!g_state.initialized) return;
    
    // 直接设置距离估计，覆盖基于面积的估计
    g_state.current_distance = clamp_float(distance, 
                                          g_config.min_distance, 
                                          g_config.max_distance);
}

void guidance_set_velocity_angle(float angle) {
    if (!g_state.initialized) return;
    
    // 更新速度夹角配置
    g_config.velocity_angle = angle;
}

void guidance_reset(void) {
    // 重置状态
    memset(&g_state, 0, sizeof(GuidanceState));
    memset(&g_current_input, 0, sizeof(GuidanceInput));
    memset(&g_current_output, 0, sizeof(GuidanceOutput));
    memset(&g_prev_output, 0, sizeof(GuidanceOutput));
    
    // 保持配置不变
    g_state.current_distance = g_config.max_distance;
    
    // 标记为未初始化
    g_state.initialized = false;
    g_state.reference_set = false;
}

GuidanceOutput guidance_get_status(void) {
    return g_current_output;
}

bool guidance_is_initialized(void) {
    return g_state.initialized;
}

bool guidance_is_reference_set(void) {
    return g_state.reference_set;
}

// 辅助函数：获取当前配置（用于调试）
GuidanceConfig guidance_get_config(void) {
    return g_config;
}

// 辅助函数：获取当前状态（用于调试）
GuidanceState guidance_get_internal_state(void) {
    return g_state;
}

// 辅助函数：设置滤波器系数
void guidance_set_filter_alpha(float alpha) {
    if (!g_state.initialized) return;
    
    g_config.filter_alpha = clamp_float(alpha, 0.0f, 1.0f);
}

// 辅助函数：设置导航比
void guidance_set_nav_ratio(float ratio) {
    if (!g_state.initialized) return;
    
    g_config.nav_ratio = clamp_float(ratio, 1.0f, 10.0f);
}
