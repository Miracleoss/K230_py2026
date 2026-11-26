#ifndef USER_CONFIG_DART_LIGHT_CONFIG_H
#define USER_CONFIG_DART_LIGHT_CONFIG_H

#include <stdint.h>
#include "Common/dart_light_status.h"

/**
 * @brief 飞镖发光装置通信与控制配置
 */
static const DartLightTeam kDartLightDefaultTeam      = DartLightTeam::Red; ///< 默认启动阵营
static const uint32_t kDartLightResponseTimeoutUs     = 5000U;              ///< 响应等待超时（5 ms，协议规定 0.1~2 ms）
static const uint32_t kDartLightInterCommandGapUs     = 500U;               ///< 指令事务间保护延时（500 us，防止总线冲突）
static const uint8_t  kDartLightSetTeamMaxRetries     = 3U;                 ///< 设置颜色失败/超时最大重试次数
static const uint32_t kDartLightSetTeamResendMs       = 2000U;              ///< 改色指令周期重发间隔（1000 ms，确保上电 3 s 窗口内送达）
static const uint32_t kDartLightStatusQueryMs         = 2000U;              ///< 装置状态周期查询间隔（1000 ms）

/**
 * @brief PC13 按键快捷切色配置
 */
static const uint32_t kDartLightKeyDebounceMs         = 20U;                ///< 按键软件消抖稳定时间（20 ms）
static const uint32_t kDartLightKeyHoldMs             = 10000U;              ///< 触发切色所需的低电平保持时间（10s）

#endif /* USER_CONFIG_DART_LIGHT_CONFIG_H */

