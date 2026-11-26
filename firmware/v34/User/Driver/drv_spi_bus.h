#ifndef USER_DRIVER_DRV_SPI_BUS_H
#define USER_DRIVER_DRV_SPI_BUS_H

#include <stdint.h>

#include "Common/common_types.h"
#include "stm32f4xx_hal.h"

/**
 * @brief SPI 总线封装类
 *
 * 对 SPI 外设进行封装，提供全双工收发接口，并将 HAL 层错误统一转换为
 * ResultCode。
 */
class Class_SpiBus {
public:
    Class_SpiBus();

    /// 绑定 SPI 句柄
    ResultCode Bind(SPI_HandleTypeDef* handle);

    /// 查询是否已绑定
    bool IsBound() const;

    /// 全双工收发指定长度的数据
    ResultCode Transfer(const uint8_t* tx, uint8_t* rx, uint16_t length, uint32_t timeout_ms);

private:
    /// 将 HAL 状态码转换为 ResultCode
    ResultCode MapHalStatus(HAL_StatusTypeDef status) const;

    SPI_HandleTypeDef* handle_;  ///< SPI 句柄指针
};

#endif /* USER_DRIVER_DRV_SPI_BUS_H */
