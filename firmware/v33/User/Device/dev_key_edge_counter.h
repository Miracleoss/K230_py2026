#ifndef USER_DEVICE_DEV_KEY_EDGE_COUNTER_H
#define USER_DEVICE_DEV_KEY_EDGE_COUNTER_H

#include <stdint.h>

#include "Common/common_types.h"
#include "Driver/drv_gpio_pin.h"

/**
 * @brief 按键边沿计数器类
 *
 * 封装 GPIO 输入引脚，提供毫秒级软件消抖以及有效边沿累计窗口管理。
 * 常用于快捷按键连击触发功能（例如 3 秒内累计 5 次按下触发切换）。
 */
class Class_KeyEdgeCounter {
public:
    Class_KeyEdgeCounter();

    /**
     * @brief 绑定按键 GPIO 引脚及计数参数
     * @param pin 目标 GPIO 引脚对象
     * @param edge_threshold 目标边沿累计阈值（如 5 次）
     * @param debounce_ms 软件消抖滤波时间（如 20 ms）
     * @param window_ms 累计有效时间窗口（如 3000 ms，首个边沿之后超时清零）
     */
    ResultCode Bind(
        Class_GpioPin* pin,
        uint8_t edge_threshold,
        uint32_t debounce_ms,
        uint32_t window_ms);

    /// 查询是否已绑定
    bool IsBound() const;

    /**
     * @brief 周期轮询按键输入（建议每毫秒调用一次）
     * @param now_ms 当前系统毫秒时间戳
     * @return 当且仅当本次更新中边沿计数达到设定阈值时返回 true，并自动复位计数
     */
    bool Update(uint32_t now_ms);

    /// 获取当前已累计的边沿次数
    uint8_t GetEdgeCount() const;

    /// 手动清空已累计的计数与窗口
    void Reset();

private:
    Class_GpioPin* pin_;
    uint8_t edge_threshold_;
    uint32_t debounce_ms_;
    uint32_t window_ms_;

    bool current_raw_state_;
    bool stable_state_;
    uint32_t last_state_change_ms_;
    uint8_t edge_count_;
    uint32_t first_edge_ms_;
};

#endif /* USER_DEVICE_DEV_KEY_EDGE_COUNTER_H */

