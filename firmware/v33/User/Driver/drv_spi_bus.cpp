#include "Driver/drv_spi_bus.h"

Class_SpiBus::Class_SpiBus()
    : handle_(nullptr)
{
}

ResultCode Class_SpiBus::Bind(SPI_HandleTypeDef* handle)
{
    if (handle == nullptr) {
        return ResultCode::ConfigError;
    }

    handle_ = handle;
    return ResultCode::Ok;
}

bool Class_SpiBus::IsBound() const
{
    return handle_ != nullptr;
}

ResultCode Class_SpiBus::Transfer(const uint8_t* tx, uint8_t* rx, uint16_t length, uint32_t timeout_ms)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    if (tx == nullptr || rx == nullptr || length == 0U) {
        return ResultCode::InvalidData;
    }

    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(handle_, const_cast<uint8_t*>(tx), rx, length, timeout_ms);
    return MapHalStatus(status);
}

ResultCode Class_SpiBus::MapHalStatus(HAL_StatusTypeDef status) const
{
    switch (status) {
    case HAL_OK:
        return ResultCode::Ok;
    case HAL_TIMEOUT:
        return ResultCode::Timeout;
    case HAL_BUSY:
        return ResultCode::NotReady;
    case HAL_ERROR:
    default:
        return ResultCode::HardwareError;
    }
}
