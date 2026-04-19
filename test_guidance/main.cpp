#include <iostream>
#include <cmath>
#include <vector>
#include "dart_guidance/include/dart_guidance.h"
#include "dart_guidance/include/guidance_math.h"

// 将欧拉角（度）转换为四元数（Z-Y-X顺序：yaw->pitch->roll）
Quaternion euler_to_quaternion(float yaw_deg, float pitch_deg, float roll_deg) {
    // 转换为弧度
    float yaw = yaw_deg * M_PI / 180.0f;
    float pitch = pitch_deg * M_PI / 180.0f;
    float roll = roll_deg * M_PI / 180.0f;

    // 计算半角
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// 生成测试数据：返回向量，每个元素为{时间戳(ms), 像素差dx, 像素差dy, 四元数}
std::vector<std::tuple<uint32_t, float, float, Quaternion>> generate_test_data() {
    std::vector<std::tuple<uint32_t, float, float, Quaternion>> data;

    // 模拟参数
    const uint32_t start_time = 1000; // 起始时间 1000ms
    const uint32_t dt = 10;           // 时间间隔 10ms (100Hz)
    const int num_frames = 100;       // 100帧数据

    // 模拟目标从中心向右下移动，然后返回
    for (int i = 0; i < num_frames; ++i) {
        uint32_t timestamp = start_time + i * dt;
        
        // 像素差：正弦变化，模拟目标在图像中移动
        float dx = 100.0f * sinf(2.0f * M_PI * i / num_frames);  // 范围 -100 到 100 像素
        float dy = 80.0f * cosf(2.0f * M_PI * i / num_frames);   // 范围 -80 到 80 像素
        
        // 姿态：轻微变化，模拟飞行器姿态扰动
        float yaw = 5.0f * sinf(0.1f * i);   // 偏航角 ±5度
        float pitch = 3.0f * cosf(0.15f * i); // 俯仰角 ±3度
        float roll = 2.0f * sinf(0.2f * i);   // 滚转角 ±2度
        
        Quaternion q = euler_to_quaternion(yaw, pitch, roll);
        
        data.emplace_back(timestamp, dx, dy, q);
    }
    
    return data;
}

int main() {
    std::cout << "=== dart_guidance 控制率验证测试 ===" << std::endl;
    std::cout << "生成测试数据..." << std::endl;
    auto test_data = generate_test_data();
    
    // 初始化制导系统
    Guidance guidance;
    
    // 配置参数
    GuidanceConfig config;
    config.nav_ratio = 4.0f;  // 导航比 4.0
    // 相机内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    config.camera_matrix[0] = 500.0f;  // fx
    config.camera_matrix[1] = 0.0f;
    config.camera_matrix[2] = 320.0f;  // cx
    config.camera_matrix[3] = 0.0f;
    config.camera_matrix[4] = 500.0f;  // fy
    config.camera_matrix[5] = 240.0f;  // cy
    config.camera_matrix[6] = 0.0f;
    config.camera_matrix[7] = 0.0f;
    config.camera_matrix[8] = 1.0f;
    config.filter_alpha = 0.3f;  // 滤波器系数
    
    // 基准四元数：假设初始时刻对准目标，姿态为零（单位四元数）
    float ref_qw = 1.0f, ref_qx = 0.0f, ref_qy = 0.0f, ref_qz = 0.0f;
    
    // 初始化制导系统
    guidance.guidance_init(config, ref_qw, ref_qx, ref_qy, ref_qz);
    
    std::cout << "开始模拟计算..." << std::endl;
    std::cout << "时间戳(ms)\t像素差dx\t像素差dy\t俯仰加速度(m/s²)\t偏航加速度(m/s²)\t有效" << std::endl;
    std::cout << "------------------------------------------------------------------------" << std::endl;
    
    // 模拟数据循环
    for (const auto& [timestamp, dx, dy, q] : test_data) {
        // 设置输入数据
        guidance.guidance_update(dx, dy, timestamp);
        
        // 设置姿态四元数（通过直接访问私有成员，这里我们假设有一个设置函数）
        // 由于guidance_update只设置了dx, dy和timestamp，我们需要设置四元数和目标速度
        // 在实际应用中，可能需要添加额外的设置函数，这里我们暂时使用默认值
        
        // 计算制导指令
        GuidanceOutput output = guidance.guidance_calculate();
        
        // 输出结果
        std::cout << timestamp << "\t\t"
                  << dx << "\t\t"
                  << dy << "\t\t"
                  << output.accel_pitch << "\t\t\t"
                  << output.accel_yaw << "\t\t\t"
                  << (output.valid ? "是" : "否") << std::endl;
    }
    
    // 测试重置功能
    std::cout << "\n测试重置功能..." << std::endl;
    guidance.guidance_reset();
    
    // 重置后尝试计算，应该返回无效输出
    GuidanceOutput reset_output = guidance.guidance_calculate();
    std::cout << "重置后输出有效: " << (reset_output.valid ? "是" : "否") << std::endl;
    
    // 重新初始化并测试
    guidance.guidance_init(config, ref_qw, ref_qx, ref_qy, ref_qz);
    guidance.guidance_update(50.0f, 30.0f, 2000);
    GuidanceOutput reinit_output = guidance.guidance_calculate();
    std::cout << "重新初始化后输出有效: " << (reinit_output.valid ? "是" : "否") << std::endl;
    std::cout << "俯仰加速度: " << reinit_output.accel_pitch << " m/s²" << std::endl;
    std::cout << "偏航加速度: " << reinit_output.accel_yaw << " m/s²" << std::endl;
    
    std::cout << "\n=== 测试完成 ===" << std::endl;
    
    return 0;
}
