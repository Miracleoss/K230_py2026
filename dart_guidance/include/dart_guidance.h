#ifndef DART_GUIDANCE_H
#define DART_GUIDANCE_H

#include "guidance_types.h"

class Guidance
{
    public:
        Guidance() = default;       
        /**
         * @brief 初始化制导系统
         * 
         * @param config 系统配置参数
         * @param ref_qw 基准四元数w分量
         * @param ref_qx 基准四元数x分量
         * @param ref_qy 基准四元数y分量
         * @param ref_qz 基准四元数z分量
         * 
         * @note 在系统启动时调用，发射前调用
         */
        void guidance_init(const GuidanceConfig& config, 
                        float ref_qw, float ref_qx, float ref_qy, float ref_qz);

        /**
         * @brief 更新视觉数据
         * 
         * @param dx x方向像素差（目标x - 图像中心x）
         * @param dy y方向像素差（目标y - 图像中心y）
         * @param timestamp 时间戳（毫秒）
         * 
         * @note 每帧图像处理完成后调用
         */
        void guidance_update(float dx, float dy, uint32_t timestamp);

        /**
         * @brief 计算制导指令（主函数）
         * 
         * @return GuidanceOutput 制导输出
         * 
         * @note 在控制周期（如100Hz）调用，返回俯仰和偏航方向的加速度指令
         */
        GuidanceOutput guidance_calculate(void);

        /**
         * @brief 重置系统状态
         * 
         * @note 在重新发射前调用，清除历史数据
         */
        void guidance_reset(void);

    private:
        //参数结构体
        GuidanceOutput g_output;
        GuidanceInput g_input;
        GuidanceConfig g_config;

        //角速度计算参数
        uint32_t pre_timestamp = 0; ///< 上一次更新的时间戳
        float prev_los_angle_yaw = 0.0; ///< 上一次的视线角
        float prev_los_angle_pitch = 0.0;

        //基础计算参数
        Quaternion reference_quat; ///< 基准四元数

        //初始化确认符
        bool is_fire = false;//fire标识符
        bool initialized = false; ///< 是否已初始化
        bool reference_set = false; ///< 是否已设置基准四元数
};



#endif /* DART_GUIDANCE_H */
