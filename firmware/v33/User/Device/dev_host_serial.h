#ifndef USER_DEVICE_DEV_HOST_SERIAL_H
#define USER_DEVICE_DEV_HOST_SERIAL_H

#include <stdint.h>

#include "Common/common_types.h"
#include "Common/control_command.h"
#include "Common/host_overload_command.h"
#include "Common/imu_data.h"
#include "Driver/drv_uart_dma.h"
#include "Protocol/host_serial_protocol.h"

class Class_HostSerial {
public:
    Class_HostSerial();

    ResultCode Bind(Class_UartDma* uart);
    ResultCode Poll();
    ResultCode TakeLatestOverload(HostOverloadCommand& command);
    ResultCode SendImuData(const ImuData& imu, const BodyControlCommand& command,
                           float overload_yaw_rate_radps, float overload_pitch_rate_radps);
    ResultCode SendVofaJustFloatData(const ImuData& imu);

    uint32_t GetCrcErrorCount() const;
    uint32_t GetFormatErrorCount() const;
    uint32_t GetRxOverflowCount() const;

private:
    Class_UartDma* uart_;
    Class_HostControlFrameParser parser_;// 上位机控制帧解析器 （串口解析处理）
    HostOverloadCommand latest_command_;
    bool command_ready_;
    uint8_t tx_buffers_[2][kHostImuFrameSize];
    uint8_t next_tx_buffer_;
    uint8_t vofa_tx_buffers_[2][kVofaJustFloatFrameSize];
    uint8_t next_vofa_tx_buffer_;
};

#endif /* USER_DEVICE_DEV_HOST_SERIAL_H */
