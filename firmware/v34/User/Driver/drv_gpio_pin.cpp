#include "Driver/drv_gpio_pin.h"

Class_GpioPin::Class_GpioPin()
    : port_(nullptr),
      pin_(0U)
{
}

ResultCode Class_GpioPin::Bind(GPIO_TypeDef* port, uint16_t pin)
{
    if (port == nullptr || pin == 0U) {
        return ResultCode::ConfigError;
    }

    port_ = port;
    pin_ = pin;
    return ResultCode::Ok;
}

bool Class_GpioPin::IsBound() const
{
    return port_ != nullptr && pin_ != 0U;
}

ResultCode Class_GpioPin::SetHigh()
{
    return Write(true);
}

ResultCode Class_GpioPin::SetLow()
{
    return Write(false);
}

ResultCode Class_GpioPin::Write(bool high)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    HAL_GPIO_WritePin(port_, pin_, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return ResultCode::Ok;
}

ResultCode Class_GpioPin::Read(bool& high) const
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    high = (HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET);
    return ResultCode::Ok;
}
