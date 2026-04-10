#include <stdio.h>
#include <math.h>
#include "dart_guidance.h"
#include "guidance_math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

int main() {
    printf("=== 相机参数和欧拉角测试 ===\n\n");
    
    // 测试1：像素到角度转换
    printf("1. 像素到角度转换测试:\n");
    float focal_length = 500.0f; // 500像素焦距
    
    float test_pixels[] = {0.0f, 100.0f, 250.0f, 500.0f, 1000.0f};
    for (int i = 0; i < 5; i++) {
        float angle = pixel_to_angle(test_pixels[i], focal_length);
        printf("   像素: %6.1f -> 角度: %6.3f rad (%6.1f°)\n", 
               test_pixels[i], angle, angle * 180.0f / M_PI);
    }
    printf("\n");
    
    // 测试2：基于相机参数的距离估计
    printf("2. 基于相机参数的距离估计:\n");
    float real_size = 0.15f; // 目标直径15cm
    float test_areas[] = {50.0f, 100.0f, 200.0f, 500.0f, 1000.0f};
    
    for (int i = 0; i < 5; i++) {
        float distance = estimate_distance_from_camera(test_areas[i], real_size, focal_length);
        printf("   像素面积: %6.1f -> 估计距离: %6.2f m\n", 
               test_areas[i], distance);
    }
    printf("\n");
    
    // 测试3：四元数到欧拉角转换（Z-Y-X顺序）
    printf("3. 四元数到欧拉角转换测试（Z-Y-X顺序）:\n");
    
    // 测试四元数：单位四元数（无旋转）
    Quaternion q_identity = {1.0f, 0.0f, 0.0f, 0.0f};
    EulerAngles angles_identity = quaternion_to_euler_zyx(q_identity);
    printf("   单位四元数 -> Yaw: %6.2f°, Pitch: %6.2f°, Roll: %6.2f°\n",
           angles_identity.yaw * 180.0f / M_PI,
           angles_identity.pitch * 180.0f / M_PI,
           angles_identity.roll * 180.0f / M_PI);
    
    // 测试四元数：绕Z轴旋转90度（偏航）
    Quaternion q_yaw90 = {0.7071f, 0.0f, 0.0f, 0.7071f}; // cos(45°), 0, 0, sin(45°)
    EulerAngles angles_yaw90 = quaternion_to_euler_zyx(q_yaw90);
    printf("   绕Z轴90° -> Yaw: %6.2f°, Pitch: %6.2f°, Roll: %6.2f°\n",
           angles_yaw90.yaw * 180.0f / M_PI,
           angles_yaw90.pitch * 180.0f / M_PI,
           angles_yaw90.roll * 180.0f / M_PI);
    
    // 测试四元数：绕Y轴旋转45度（俯仰）
    Quaternion q_pitch45 = {0.9239f, 0.0f, 0.3827f, 0.0f}; // cos(22.5°), 0, sin(22.5°), 0
    EulerAngles angles_pitch45 = quaternion_to_euler_zyx(q_pitch45);
    printf("   绕Y轴45° -> Yaw: %6.2f°, Pitch: %6.2f°, Roll: %6.2f°\n",
           angles_pitch45.yaw * 180.0f / M_PI,
           angles_pitch45.pitch * 180.0f / M_PI,
           angles_pitch45.roll * 180.0f / M_PI);
    
    printf("\n");
    
    // 测试4：完整的制导系统测试
    printf("4. 完整制导系统测试（使用相机参数）:\n");
    
    GuidanceConfig config = {
        .nav_ratio = 4.0f,
        .min_distance = 5.0f,
        .max_distance = 25.0f,
        .min_area = 10.0f,
        .max_area = 1000.0f,
        .focal_length = 500.0f,
        .target_real_size = 0.15f, // 使用相机参数方法
        .velocity_angle = 0.1f,
        .filter_alpha = 0.3f
    };
    
    // 初始化系统
    guidance_init(config, 1.0f, 0.0f, 0.0f, 0.0f);
    
    // 模拟视觉数据
    float pixel_dx = 100.0f;
    float pixel_dy = 50.0f;
    float pixel_area = 300.0f;
    
    // 更新数据
    guidance_update_vision(pixel_dx, pixel_dy, pixel_area, 1000);
    guidance_update_attitude(0.707f, 0.0f, 0.707f, 0.0f, 1000);
    
    // 计算制导指令
    GuidanceOutput output = guidance_calculate();
    
    if (output.valid) {
        printf("   输入: dx=%.1f, dy=%.1f, area=%.1f\n", pixel_dx, pixel_dy, pixel_area);
        printf("   输出: 距离=%.2f m, 视线角速度=%.4f rad/s\n", 
               output.est_distance, output.los_rate);
        printf("         俯仰加速度=%.2f m/s², 偏航加速度=%.2f m/s²\n",
               output.accel_pitch, output.accel_yaw);
    } else {
        printf("   制导计算失败\n");
    }
    
    printf("\n=== 测试完成 ===\n");
    
    return 0;
}
