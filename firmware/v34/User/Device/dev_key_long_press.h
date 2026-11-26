#ifndef USER_DEVICE_DEV_KEY_LONG_PRESS_H
#define USER_DEVICE_DEV_KEY_LONG_PRESS_H

#include <stdint.h>

#include "Common/common_types.h"
#include "Driver/drv_gpio_pin.h"

/**
 * @brief 按键长按检测类
 *
 * 封装 GPIO 输入引脚，提供毫秒级软件消抖与低电平保持时长检测。
 * 常用于长按触发功能（例如持续按下 1500 ms 触发切色）。
 */
class Class_KeyLongPress {
public:
    Class_KeyLongPress();

    /**
     * @brief 绑定按键 GPIO 引脚及检测参数
     * @param pin 目标 GPIO 引脚对象
     * @param debounce_ms 软件消抖滤波时间（如 20 ms）
     * @param hold_ms 低电平保持触发阈值（如 1500 ms）
     */
    ResultCode Bind(
        Class_GpioPin* pin,
        uint32_t debounce_ms,
        uint32_t hold_ms);

    /// 查询是否已绑定
    bool IsBound() const;

    /**
     * @brief 周期轮询按键输入（建议每毫秒调用一次）
     * @param now_ms 当前系统毫秒时间戳
     * @return 当且仅当本次更新中低电平保持达到阈值时返回 true（每次按下仅触发一次，松开后重新计时）
     */
    bool Update(uint32_t now_ms);

    /// 当前消抖后的按键是否处于按下（低电平）状态
    bool IsHeldLow() const;

private:
    Class_GpioPin* pin_;
    uint32_t debounce_ms_;
    uint32_t hold_ms_;

    bool current_raw_state_;
    bool stable_state_;
    uint32_t last_state_change_ms_;
    uint32_t low_start_ms_;
    bool reported_;
};

#endif /* USER_DEVICE_DEV_KEY_LONG_PRESS_H */
