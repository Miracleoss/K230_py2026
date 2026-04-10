#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "dart_guidance.h"

// 模拟数据生成函数
float generate_pixel_dx(int iteration) {
    // 模拟目标在图像中移动：从右侧移动到左侧
    return 100.0f - iteration * 2.0f;
}

float generate_pixel_dy(int iteration) {
    // 模拟目标在垂直方向轻微移动
    return 50.0f + sin(iteration * 0.1f) * 20.0f;
}

float generate_pixel_area(int iteration) {
    // 模拟面积逐渐减小（距离增加）
    return 500.0f - iteration * 2.0f;
}

void generate_quaternion(int iteration, float* qw, float* qx, float* qy, float* qz) {
    // 模拟姿态缓慢变化
    float angle = iteration * 0.01f; // 缓慢旋转
    
    // 绕z轴旋转
    *qw = cos(angle / 2.0f);
    *qx = 0.0f;
    *qy = 0.0f;
    *qz = sin(angle / 2.0f);
}

int main() {
    printf("=== 飞镖制导库示例程序 ===\n\n");
    
    // 1. 配置参数
    GuidanceConfig config = {
        .nav_ratio = 4.0f,           // 导航比 4.0
        .min_distance = 5.0f,        // 最小距离 5m
        .max_distance = 25.0f,       // 最大距离 25m
        .min_area = 10.0f,           // 最小像素面积
        .max_area = 1000.0f,         // 最大像素面积
        .focal_length = 500.0f,      // 焦距 500像素
        .velocity_angle = 0.1f,      // 速度夹角 0.1rad
        .filter_alpha = 0.3f         // 滤波器系数 0.3
    };
    
    printf("配置参数:\n");
    printf("  导航比: %.1f\n", config.nav_ratio);
    printf("  距离范围: %.1f - %.1f m\n", config.min_distance, config.max_distance);
    printf("  像素面积范围: %.1f - %.1f\n", config.min_area, config.max_area);
    printf("  焦距: %.1f 像素\n", config.focal_length);
    printf("  速度夹角: %.3f rad\n", config.velocity_angle);
    printf("  滤波器系数: %.2f\n\n", config.filter_alpha);
    
    // 2. 初始化（假设初始姿态为单位四元数）
    printf("初始化制导系统...\n");
    guidance_init(config, 1.0f, 0.0f, 0.0f, 0.0f);
    
    if (!guidance_is_initialized()) {
        printf("错误: 系统初始化失败!\n");
        return 1;
    }
    printf("系统初始化成功\n\n");
    
    // 3. 模拟发射瞬间：设置基准四元数
    printf("模拟发射瞬间...\n");
    float launch_qw = 0.707f, launch_qx = 0.0f, launch_qy = 0.707f, launch_qz = 0.0f;
    guidance_set_reference_quaternion(launch_qw, launch_qx, launch_qy, launch_qz);
    
    if (!guidance_is_reference_set()) {
        printf("错误: 基准四元数设置失败!\n");
        return 1;
    }
    printf("基准四元数设置成功: [%.3f, %.3f, %.3f, %.3f]\n\n", 
           launch_qw, launch_qx, launch_qy, launch_qz);
    
    // 4. 主循环（模拟飞行过程）
    printf("开始模拟飞行过程...\n");
    printf("迭代 | 像素差(dx,dy) | 面积 | 距离(m) | 视线角速度(rad/s) | 俯仰加速度(m/s²) | 偏航加速度(m/s²)\n");
    printf("-----|----------------|------|---------|-------------------|-------------------|-----------------\n");
    
    for (int i = 0; i < 20; i++) {
        uint32_t current_time = i * 100; // 模拟100ms周期
        
        // 生成模拟数据
        float pixel_dx = generate_pixel_dx(i);
        float pixel_dy = generate_pixel_dy(i);
        float pixel_area = generate_pixel_area(i);
        
        float qw, qx, qy, qz;
        generate_quaternion(i, &qw, &qx, &qy, &qz);
        
        // 5. 更新数据
        guidance_update_vision(pixel_dx, pixel_dy, pixel_area, current_time);
        guidance_update_attitude(qw, qx, qy, qz, current_time);
        
        // 6. 计算制导指令
        GuidanceOutput output = guidance_calculate();
        
        // 7. 显示结果
        if (output.valid) {
            printf("%4d | (%6.1f, %6.1f) | %5.0f | %7.1f | %17.4f | %17.2f | %16.2f\n",
                   i, pixel_dx, pixel_dy, pixel_area,
                   output.est_distance, output.los_rate,
                   output.accel_pitch, output.accel_yaw);
        } else {
            printf("%4d | (%6.1f, %6.1f) | %5.0f | %7s | %17s | %17s | %16s\n",
                   i, pixel_dx, pixel_dy, pixel_area,
                   "无效", "无效", "无效", "无效");
        }
        
        // 模拟一些变化
        if (i == 5) {
            // 模拟更新速度夹角
            guidance_set_velocity_angle(0.15f);
            printf("  [迭代 %d] 更新速度夹角为 0.15 rad\n", i);
        }
        
        if (i == 10) {
            // 模拟设置外部距离估计
            guidance_set_distance(15.0f);
            printf("  [迭代 %d] 设置外部距离估计为 15.0 m\n", i);
        }
    }
    
    // 8. 获取最终状态
    printf("\n最终状态:\n");
    GuidanceOutput final_status = guidance_get_status();
    if (final_status.valid) {
        printf("  估计距离: %.1f m\n", final_status.est_distance);
        printf("  视线角速度: %.4f rad/s\n", final_status.los_rate);
        printf("  俯仰加速度: %.2f m/s²\n", final_status.accel_pitch);
        printf("  偏航加速度: %.2f m/s²\n", final_status.accel_yaw);
    }
    
    // 9. 重置系统
    printf("\n重置系统...\n");
    guidance_reset();
    
    if (!guidance_is_initialized()) {
        printf("系统已成功重置，准备下一次发射\n");
    }
    
    printf("\n=== 示例程序结束 ===\n");
    
    return 0;
}
