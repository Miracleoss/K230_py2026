#ifndef USER_COMMON_DART_LIGHT_STATUS_H
#define USER_COMMON_DART_LIGHT_STATUS_H

#include <stdint.h>

/**
 * @brief 飞镖发光装置阵营颜色
 */
enum class DartLightTeam : uint8_t {
    Red  = 0x00, ///< 红方
    Blue = 0x01  ///< 蓝方
};

/**
 * @brief 飞镖发光装置状态与错误码数据
 */
struct DartLightStatus {
    uint8_t version;              ///< 固件版本（1～255）
    int8_t temp_deg_c;            ///< PCB 板温度，单位 ℃
    uint8_t raw_state;            ///< 原始状态字节
    DartLightTeam team;           ///< 阵营信息（Bit3: 0 红方, 1 蓝方）
    bool visible_led_on;          ///< 可见光灯珠状态（Bit2: 0 熄灭, 1 点亮）
    bool supply_voltage_low;      ///< 供电电压状态（Bit1: 0 正常, 1 电压过低）
    bool ir_led_fault;            ///< 红外灯珠状态（Bit0: 0 正常, 1 异常）
    uint64_t update_timestamp_us; ///< 收到并解码响应时的系统微秒时间戳
    bool valid;                   ///< 数据有效标记
};

#endif /* USER_COMMON_DART_LIGHT_STATUS_H */

