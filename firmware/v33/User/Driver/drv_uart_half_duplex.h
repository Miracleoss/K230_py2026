#ifndef USER_DRIVER_DRV_UART_HALF_DUPLEX_H
#define USER_DRIVER_DRV_UART_HALF_DUPLEX_H

#include <stdint.h>

#include "Common/common_types.h"
#include "stm32f4xx_hal.h"

/**
 * @brief 单线半双工 UART 中断驱动类
 *
 * 专用于单线半双工通信（如 UART4 PA0）。
 * 发送时自动切为 TX 并禁止 RE，发送完成（TC 中断）后立刻切回 RX 并启动接收，
 * 接收采用逐字节中断入环形缓冲，提供非阻塞收发接口。
 */
class Class_UartHalfDuplex {
public:
    Class_UartHalfDuplex();

    /// 绑定底层 UART 句柄
    ResultCode Bind(UART_HandleTypeDef* uart);

    /// 查询是否已绑定
    bool IsBound() const;

    /// 开启中断接收模式，并指定接收环形缓冲区及容量
    ResultCode StartReceive(uint8_t* buffer, uint16_t capacity);

    /// 异步发送一帧数据（切换为 TX 模式并启动中断发送）
    ResultCode SendAsync(const uint8_t* data, uint16_t length);

    /// 查询当前是否处于发送忙碌状态
    bool IsTxBusy() const;

    /// 获取环形缓冲区中可读字节数
    uint32_t Available() const;

    /// 读取一个字节
    ResultCode ReadByte(uint8_t& byte);

    /// 清空接收环形缓冲区
    void FlushRx();

    /// 发送完成中断回调处理（由 HAL_UART_TxCpltCallback 转发）
    void OnTxCpltIsr();

    /// 接收完成中断回调处理（由 HAL_UART_RxCpltCallback 转发）
    void OnRxCpltIsr();

    /// 错误中断回调处理（由 HAL_UART_ErrorCallback 转发）
    void OnErrorIsr(uint32_t error_code);

    // 诊断统计接口
    uint32_t GetTxCount() const;
    uint32_t GetRxByteCount() const;
    uint32_t GetRxOverflowCount() const;
    uint32_t GetRxErrorCount() const;
    uint32_t GetLastRxError() const;

private:
    ResultCode MapHalStatus(HAL_StatusTypeDef status) const;

    UART_HandleTypeDef* uart_;
    uint8_t* rx_buffer_;
    uint16_t rx_capacity_;
    volatile uint16_t rx_write_position_;
    uint16_t rx_read_position_;
    uint8_t rx_byte_;

    uint8_t tx_buffer_[16];
    volatile bool is_tx_busy_;

    volatile uint32_t tx_count_;
    volatile uint32_t rx_byte_count_;
    volatile uint32_t rx_overflow_count_;
    volatile uint32_t rx_error_count_;
    volatile uint32_t rx_last_error_;
};

#endif /* USER_DRIVER_DRV_UART_HALF_DUPLEX_H */

