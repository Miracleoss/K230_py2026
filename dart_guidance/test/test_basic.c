#include <stdio.h>
#include <math.h>
#include "dart_guidance.h"

// 测试基本功能
void test_basic_functionality() {
    printf("=== 基本功能测试 ===\n");
    
    // 1. 初始化测试
    GuidanceConfig config = {
        .nav_ratio = 4.0f,
        .min_distance = 5.0f,
        .max_distance = 25.0f,
        .min_area = 10.0f,
        .max_area = 1000.0f,
        .focal_length = 500.0f,
        .velocity_angle = 0.1f,
        .filter_alpha = 0.3f
    };
    
    guidance_init(config, 1.0f, 0.0f, 0.0f, 0.0f);
    
    if (guidance_is_initialized()) {
        printf("✓ 初始化成功\n");
    } else {
        printf("✗ 初始化失败\n");
        return;
    }
    
    // 2. 设置基准四元数测试
    guidance_set_reference_quaternion(0.707f, 0.0f, 0.707f, 0.0f);
    
    if (guidance_is_reference_set()) {
        printf("✓ 基准四元数设置成功\n");
    } else {
        printf("✗ 基准四元数设置失败\n");
    }
    
    // 3. 更新数据测试
    guidance_update_vision(50.0f, 30.0f, 200.0f, 1000);
    guidance_update_attitude(0.707f, 0.0f, 0.707f, 0.0f, 1000);
    printf("✓ 数据更新成功\n");
    
    // 4. 计算测试
    GuidanceOutput output = guidance_calculate();
    
    if (output.valid) {
        printf("✓ 制导计算成功\n");
        printf("  估计距离: %.2f m\n", output.est_distance);
        printf("  视线角速度: %.4f rad/s\n", output.los_rate);
        printf("  俯仰加速度: %.2f m/s²\n", output.accel_pitch);
        printf("  偏航加速度: %.2f m/s²\n", output.accel_yaw);
    } else {
        printf("✗ 制导计算失败\n");
    }
    
    // 5. 重置测试
    guidance_reset();
    
    if (!guidance_is_initialized()) {
        printf("✓ 系统重置成功\n");
    } else {
        printf("✗ 系统重置失败\n");
    }
    
    printf("\n");
}

// 测试距离估计
void test_distance_estimation() {
    printf("=== 距离估计测试 ===\n");
    
    GuidanceConfig config = {
        .nav_ratio = 4.0f,
        .min_distance = 5.0f,
        .max_distance = 25.0f,
        .min_area = 100.0f,
        .max_area = 1000.0f,
        .focal_length = 500.0f,
        .velocity_angle = 0.1f,
        .filter_alpha = 0.3f
    };
    
    guidance_init(config, 1.0f, 0.0f, 0.0f, 0.0f);
    guidance_set_reference_quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    
    // 测试不同面积对应的距离估计
    float test_areas[] = {100.0f, 300.0f, 550.0f, 800.0f, 1000.0f};
    int num_tests = sizeof(test_areas) / sizeof(test_areas[0]);
    
    for (int i = 0; i < num_tests; i++) {
        guidance_update_vision(0.0f, 0.0f, test_areas[i], i * 100);
        guidance_update_attitude(1.0f, 0.0f, 0.0f, 0.0f, i * 100);
        
        GuidanceOutput output = guidance_calculate();
        
        if (output.valid) {
            printf("面积: %6.1f -> 估计距离: %6.2f m", test_areas[i], output.est_distance);
            
            // 验证距离估计是否在合理范围内
            if (output.est_distance >= config.min_distance && 
                output.est_distance <= config.max_distance) {
                printf(" ✓\n");
            } else {
                printf(" ✗ (超出范围)\n");
            }
        }
    }
    
    guidance_reset();
    printf("\n");
}

// 测试加速度输出范围
void test_acceleration_range() {
    printf("=== 加速度输出范围测试 ===\n");
    
    GuidanceConfig config = {
        .nav_ratio = 4.0f,
        .min_distance = 10.0f,
        .max_distance = 20.0f,
        .min_area = 100.0f,
        .max_area = 1000.0f,
        .focal_length = 500.0f,
        .velocity_angle = 0.0f,
        .filter_alpha = 0.3f
    };
    
    guidance_init(config, 1.0f, 0.0f, 0.0f, 0.0f);
    guidance_set_reference_quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    
    // 测试不同视线角速度下的加速度输出
    float test_rates[] = {-1.0f, -0.5f, -0.1f, 0.0f, 0.1f, 0.5f, 1.0f};
    int num_tests = sizeof(test_rates) / sizeof(test_rates[0]);
    
    printf("视线角速度(rad/s) | 俯仰加速度(m/s²) | 偏航加速度(m/s²) | 是否在范围内\n");
    printf("------------------|------------------|------------------|-------------\n");
    
    for (int i = 0; i < num_tests; i++) {
        // 模拟产生特定角速度的像素差变化
        float simulated_dx = test_rates[i] * 100.0f; // 简化模拟
        
        guidance_update_vision(simulated_dx, 0.0f, 500.0f, i * 100);
        guidance_update_attitude(1.0f, 0.0f, 0.0f, 0.0f, i * 100);
        
        GuidanceOutput output = guidance_calculate();
        
        if (output.valid) {
            // 检查加速度是否在合理范围内（假设最大50 m/s²）
            float max_accel = 50.0f;
            int pitch_in_range = fabsf(output.accel_pitch) <= max_accel;
            int yaw_in_range = fabsf(output.accel_yaw) <= max_accel;
            
            printf("%17.2f | %16.2f | %16.2f | %s\n",
                   test_rates[i], output.accel_pitch, output.accel_yaw,
                   (pitch_in_range && yaw_in_range) ? "✓" : "✗");
        }
    }
    
    guidance_reset();
    printf("\n");
}

// 测试与Python视觉模块的集成示例
void test_python_integration_example() {
    printf("=== Python集成示例 ===\n");
    
    // 模拟Python视觉模块的输出
    typedef struct {
        float cx;      // 目标中心x
        float cy;      // 目标中心y
        float area;    // 目标面积
        uint32_t timestamp;
    } PythonBlob;
    
    // 模拟Python IMU模块的输出
    typedef struct {
        float qw, qx, qy, qz;
        uint32_t timestamp;
    } PythonIMU;
    
    // 模拟数据
    PythonBlob blob = {320.0f, 240.0f, 150.0f, 1000};
    PythonIMU imu = {0.707f, 0.0f, 0.707f, 0.0f, 1000};
    
    // 相机参数（应与Python端一致）
    float image_center_x = 320.0f;
    float image_center_y = 240.0f;
    
    // 计算像素差
    float pixel_dx = blob.cx - image_center_x;
    float pixel_dy = blob.cy - image_center_y;
    
    printf("Python视觉模块输出:\n");
    printf("  目标位置: (%.1f, %.1f)\n", blob.cx, blob.cy);
    printf("  目标面积: %.1f\n", blob.area);
    printf("  像素差: (%.1f, %.1f)\n", pixel_dx, pixel_dy);
    printf("  IMU四元数: [%.3f, %.3f, %.3f, %.3f]\n\n", 
           imu.qw, imu.qx, imu.qy, imu.qz);
    
    // 初始化制导系统
    GuidanceConfig config = {
        .nav_ratio = 4.0f,
        .min_distance = 5.0f,
        .max_distance = 25.0f,
        .min_area = 10.0f,
        .max_area = 1000.0f,
        .focal_length = 500.0f,
        .velocity_angle = 0.1f,
        .filter_alpha = 0.3f
    };
    
    guidance_init(config, 1.0f, 0.0f, 0.0f, 0.0f);
    guidance_set_reference_quaternion(imu.qw, imu.qx, imu.qy, imu.qz);
    
    // 更新数据（模拟Python调用C库）
    guidance_update_vision(pixel_dx, pixel_dy, blob.area, blob.timestamp);
    guidance_update_attitude(imu.qw, imu.qx, imu.qy, imu.qz, imu.timestamp);
    
    // 计算制导指令
    GuidanceOutput output = guidance_calculate();
    
    if (output.valid) {
        printf("C库制导输出:\n");
        printf("  估计距离: %.2f m\n", output.est_distance);
        printf("  视线角速度: %.4f rad/s\n", output.los_rate);
        printf("  俯仰加速度: %.2f m/s²\n", output.accel_pitch);
        printf("  偏航加速度: %.2f m/s²\n", output.accel_yaw);
        
        printf("\nPython端后续处理:\n");
        printf("  # 将加速度传递给控制器\n");
        printf("  control_pitch(%.2f)\n", output.accel_pitch);
        printf("  control_yaw(%.2f)\n", output.accel_yaw);
    }
    
    guidance_reset();
    printf("\n");
}

int main() {
    printf("飞镖制导库测试程序\n");
    printf("==================\n\n");
    
    test_basic_functionality();
    test_distance_estimation();
    test_acceleration_range();
    test_python_integration_example();
    
    printf("所有测试完成！\n");
    
    return 0;
}
