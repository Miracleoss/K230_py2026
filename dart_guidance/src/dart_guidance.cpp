#include "dart_guidance.h"
#include "guidance_math.h"
#include <string.h>
#include <math.h>


// 默认配置
static const GuidanceConfig DEFAULT_CONFIG = {
    .nav_ratio = 4.0f,           // 导航比 4.0
    .camera_matrix = {500.0f, 0.0f, 320.0f, 0.0f, 500.0f, 240.0f, 0.0f, 0.0f, 1.0f}, // 焦距 500像素
    .filter_alpha = 0.3f,        // 滤波器系数 0.3
    .max_accel_yaw = 100.0f,     // 偏航最大加速度 100 m/s²
    .max_accel_pitch = 100.0f    // 俯仰最大加速度 100 m/s²
};

void Guidance::guidance_init(const GuidanceConfig& config, 
                   float ref_qw, float ref_qx, float ref_qy, float ref_qz) {
    this->g_config = config;
    this->reference_quat.w = ref_qw;
    this->reference_quat.x = ref_qx;
    this->reference_quat.y = ref_qy;
    this->reference_quat.z = ref_qz;
    
    // 归一化基准四元数
    this->reference_quat = quaternion_normalize(this->reference_quat);
    
    // 初始化状态
    this->initialized = true;
    this->reference_set = true;
    this->is_fire = false;
    this->pre_timestamp = 0;
    this->prev_los_angle_yaw = 0.0f;
    this->prev_los_angle_pitch = 0.0f;
    
    
    // 初始化输入
    this->g_input.pixel_dx = 0.0f;
    this->g_input.pixel_dy = 0.0f;
    this->g_input.q = (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};
    this->g_input.target_velocity = 0.0f;
    this->g_input.timestamp = 0;
}




void Guidance::fire(){
    this->is_fire = true;
}

void Guidance::guidance_update(float dx, float dy,float qw, float qx, float qy, float qz,float target_velocity, uint32_t timestamp) {
    // 更新输入数据
    this->g_input.pixel_dx = dx;
    this->g_input.pixel_dy = dy;
    this->g_input.timestamp = timestamp;

    this->g_input.q.w = qw;
    this->g_input.q.x = qx;
    this->g_input.q.y = qy;
    this->g_input.q.z = qz;

    this->g_input.q = quaternion_normalize(this->g_input.q);
    
    this->g_input.target_velocity = target_velocity;
}

GuidanceOutput Guidance::guidance_calculate(void) {
    if(!is_fire){
        GuidanceOutput invalid_output = {0};
        invalid_output.valid = false;
        return invalid_output;
    }
    if (!this->initialized || !this->reference_set) {
        GuidanceOutput invalid_output = {0};
        invalid_output.valid = false;
        return invalid_output;
    }
    
    // 检查输入数据是否有效
    if (this->g_input.timestamp == 0) {
        GuidanceOutput invalid_output = {0};
        invalid_output.valid = false;
        return invalid_output;
    }
    
    // 调用核心算法
    // 像素坐标系->相机坐标系
    Cordation camera;
    camera.x = (this->g_input.pixel_dx - this->g_config.camera_matrix[2]) / this->g_config.camera_matrix[0];
    camera.y = (this->g_input.pixel_dy - this->g_config.camera_matrix[5]) / this->g_config.camera_matrix[4];
    camera.z = 1.0f;

    // 相机坐标系->镖体系(如果不准可以加入手眼标定)
    Cordation dart;
    dart.x = camera.z;
    dart.y = camera.x;
    dart.z = camera.y;

    // 镖体系->惯性系
    Quaternion rel_q = quaternion_relative(this->g_input.q, this->reference_quat);
    
    // 从四元数计算旋转矩阵（弹体到惯性系）
    float C00 = 1.0f - 2.0f * (rel_q.y * rel_q.y + rel_q.z * rel_q.z);
    float C01 = 2.0f * (rel_q.x * rel_q.y - rel_q.w * rel_q.z);
    float C02 = 2.0f * (rel_q.x * rel_q.z + rel_q.w * rel_q.y);
    
    float C10 = 2.0f * (rel_q.x * rel_q.y + rel_q.w * rel_q.z);
    float C11 = 1.0f - 2.0f * (rel_q.x * rel_q.x + rel_q.z * rel_q.z);
    float C12 = 2.0f * (rel_q.y * rel_q.z - rel_q.w * rel_q.x);
    
    float C20 = 2.0f * (rel_q.x * rel_q.z - rel_q.w * rel_q.y);
    float C21 = 2.0f * (rel_q.y * rel_q.z + rel_q.w * rel_q.x);
    float C22 = 1.0f - 2.0f * (rel_q.x * rel_q.x + rel_q.y * rel_q.y);
    
    // 使用旋转矩阵将镖体系坐标转换到惯性系
    Cordation inertial;
    inertial.x = C00 * dart.x + C01 * dart.y + C02 * dart.z;
    inertial.y = C10 * dart.x + C11 * dart.y + C12 * dart.z;
    inertial.z = C20 * dart.x + C21 * dart.y + C22 * dart.z;
    
    // 计算视线角
    float los_angle_yaw = safe_atan2(inertial.y, inertial.x);
    float los_angle_pitch = safe_atan2(-inertial.z, sqrtf(inertial.x * inertial.x + inertial.y * inertial.y));
    
    // 检查时间戳是否合理（防止除零错误）
    if (this->pre_timestamp == 0) {
        // 第一次计算：记录当前时间戳和视线角，返回零输出
        // 需要等到下一次调用才能计算角速度
        this->pre_timestamp = this->g_input.timestamp;
        this->prev_los_angle_yaw = los_angle_yaw;
        this->prev_los_angle_pitch = los_angle_pitch;
        
        GuidanceOutput zero_output = {0};
        zero_output.valid = true;
        return zero_output;
    }
    
    // 计算时间差（毫秒转换为秒）
    float dt = (float)(this->g_input.timestamp - this->pre_timestamp) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) { // 防止无效时间差
        GuidanceOutput output;
        output.valid = false;
        return output;
    }
    
    // 计算视线角速度
    float los_angle_yaw_velocity = (los_angle_yaw - this->prev_los_angle_yaw) / dt;
    float los_angle_pitch_velocity = (los_angle_pitch - this->prev_los_angle_pitch) / dt;
    
    // 应用低通滤波器平滑角速度
    los_angle_yaw_velocity = low_pass_filter(los_angle_yaw_velocity, this->prev_los_angle_yaw_velocity, this->g_config.filter_alpha);
    los_angle_pitch_velocity = low_pass_filter(los_angle_pitch_velocity, this->prev_los_angle_pitch_velocity, this->g_config.filter_alpha);
    
    // 计算制导指令（比例导引）
    float n_yaw = this->g_config.nav_ratio * this->g_input.target_velocity * los_angle_yaw_velocity;
    float n_pitch = this->g_config.nav_ratio * this->g_input.target_velocity * los_angle_pitch_velocity;
    
    // 限制加速度范围
    n_yaw = clamp_float(n_yaw, -this->g_config.max_accel_yaw, this->g_config.max_accel_yaw);
    n_pitch = clamp_float(n_pitch, -this->g_config.max_accel_pitch, this->g_config.max_accel_pitch);
    
    // 更新历史数据
    this->prev_los_angle_yaw = los_angle_yaw;
    this->prev_los_angle_pitch = los_angle_pitch;
    this->prev_los_angle_pitch_velocity = los_angle_pitch_velocity;
    this->prev_los_angle_yaw_velocity = los_angle_yaw_velocity;
    this->pre_timestamp = this->g_input.timestamp;
    
    // 准备输出
    GuidanceOutput output;
    output.accel_pitch = n_pitch;
    output.accel_yaw = n_yaw;
    output.valid = true;
    
    return output;
}


void Guidance::guidance_reset(void) {
    this->initialized = false;
    this->reference_set = false;
    this->is_fire = false;
    this->pre_timestamp = 0;
    this->prev_los_angle_yaw = 0.0f;
    this->prev_los_angle_pitch = 0.0f;
    
    // 重置输入
    this->g_input.pixel_dx = 0.0f;
    this->g_input.pixel_dy = 0.0f;
    this->g_input.q = (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};
    this->g_input.target_velocity = 0.0f;
    this->g_input.timestamp = 0;
}
