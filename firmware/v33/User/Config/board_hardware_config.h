#ifndef USER_CONFIG_BOARD_HARDWARE_CONFIG_H
#define USER_CONFIG_BOARD_HARDWARE_CONFIG_H

/**
 * @brief PWM 通道标识
 *
 * 对应板载定时器的具体 PWM 输出通道。
 */
enum class PwmChannelId {
    Unassigned, ///< 未分配
    Tim1Ch1,    ///< TIM1 通道 1
    Tim1Ch2,    ///< TIM1 通道 2
    Tim1Ch3,    ///< TIM1 通道 3
    Tim1Ch4,    ///< TIM1 通道 4
    Tim3Ch3     ///< TIM3 通道 3
};

/**
 * @brief SPI 总线标识
 */
enum class SpiBusId {
    Spi1 ///< SPI1
};

/**
 * @brief UART 串口标识
 */
enum class UartPortId {
    Usart1, ///< USART1
    Uart4   ///< UART4
};

/**
 * @brief 定时器标识
 */
enum class TimerId {
    Tim5 ///< TIM5
};

/**
 * @brief 判断指定 PWM 通道是否为舵机通道
 * @param pwm_channel_id PWM 通道标识
 * @return true 表示为舵机通道（TIM1 的 4 个通道），false 表示否
 */
inline bool IsServoPwmChannel(PwmChannelId pwm_channel_id)
{
    return pwm_channel_id == PwmChannelId::Tim1Ch1 ||
           pwm_channel_id == PwmChannelId::Tim1Ch2 ||
           pwm_channel_id == PwmChannelId::Tim1Ch3 ||
           pwm_channel_id == PwmChannelId::Tim1Ch4;
}

/**
 * @brief 获取加热器对应的 PWM 通道标识
 * @return 加热器 PWM 通道（TIM3 通道 3）
 */
inline PwmChannelId GetHeaterPwmChannelId()
{
    return PwmChannelId::Tim3Ch3;
}

/**
 * @brief 获取 BMI088 惯性传感器使用的 SPI 总线标识
 * @return SPI 总线（SPI1）
 */
inline SpiBusId GetBmi088SpiBusId()
{
    return SpiBusId::Spi1;
}

/**
 * @brief 获取 A133P 上位机通信使用的串口标识
 * @return UART 串口（USART1）
 */
inline UartPortId GetA133pUartPortId()
{
    return UartPortId::Usart1;
}

/**
 * @brief 获取飞镖发光装置通信使用的串口标识
 * @return UART 串口（UART4）
 */
inline UartPortId GetDartLightUartPortId()
{
    return UartPortId::Uart4;
}

/**
 * @brief 获取调度器使用的定时器标识
 * @return 调度器定时器（TIM5）
 */
inline TimerId GetSchedulerTimerId()
{
    return TimerId::Tim5;
}

#endif /* USER_CONFIG_BOARD_HARDWARE_CONFIG_H */
