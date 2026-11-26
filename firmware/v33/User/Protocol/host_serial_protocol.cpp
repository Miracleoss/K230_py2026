/**
 * @file host_serial_protocol.cpp
 * @brief 主机控制帧解码与 IMU 帧编码的实现。
 */
#include "Protocol/host_serial_protocol.h"

#include <float.h>
#include <string.h>

static const uint8_t kVofaJustFloatTail0 = 0x00U;
static const uint8_t kVofaJustFloatTail1 = 0x00U;
static const uint8_t kVofaJustFloatTail2 = 0x80U;
static const uint8_t kVofaJustFloatTail3 = 0x7FU;

static const uint8_t kControlHeader0 = 0xAAU;  ///< 控制帧的第一个帧头字节。
static const uint8_t kControlHeader1 = 0x55U;  ///< 控制帧的第二个帧头字节。
static const uint8_t kImuHeader0 = 0x55U;      ///< IMU 帧的第一个帧头字节。
static const uint8_t kImuHeader1 = 0xAAU;      ///< IMU 帧的第二个帧头字节。
static const uint8_t kFooter0 = 0x0DU;         ///< 回车符帧尾字节。
static const uint8_t kFooter1 = 0x0AU;         ///< 换行符帧尾字节。
static const uint8_t kImuTriggerFlagPlaceholder = 1U;  ///< IMU 帧相机硬触发标志占位值（当前恒为触发）。

/**
 * @brief 从按小端序排列的线上字节中读取一个有符号 16 位整数。
 * @param data 指向两个可读字节的指针。
 * @return 解码后的有符号值。
 */
static int16_t ReadInt16Le(const uint8_t* data)
{
    const uint16_t raw = static_cast<uint16_t>(data[0]) |
                         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
    return static_cast<int16_t>(raw);
}

/**
 * @brief 按小端序线上字节顺序写入一个有符号 16 位整数。
 * @param value 要编码的有符号值。
 * @param data 指向可容纳两个字节的目标缓冲区。
 */
static void WriteInt16Le(int16_t value, uint8_t* data)
{
    const uint16_t raw = static_cast<uint16_t>(value);
    data[0] = static_cast<uint8_t>(raw & 0xFFU);
    data[1] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
}

static void WriteInt32Le(uint32_t value, uint8_t* data)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

static bool IsFiniteFloat(float value)
{
    return value == value && value >= -FLT_MAX && value <= FLT_MAX;
}

static void WriteFloatLe(float value, uint8_t* data)
{
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    data[0] = static_cast<uint8_t>(bits & 0xFFU);
    data[1] = static_cast<uint8_t>((bits >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((bits >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((bits >> 24U) & 0xFFU);
}

/**
 * @brief 将物理浮点值转换为四舍五入后的 int16 线上值。
 * @param value 物理输入值。
 * @param units_per_value 每个物理单位对应的编码 LSB 数量。
 * @param encoded 量化结果的输出目标。
 * @return 当缩放后的值为有限值且处于 int16 范围内时返回 true。
 */
static bool QuantizeInt16(float value, float units_per_value, int16_t& encoded)
{
    const float scaled = value * units_per_value;
    if (!(scaled >= -32768.0f && scaled <= 32767.0f)) {
        return false;
    }

    const int32_t rounded = static_cast<int32_t>(
        scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
    encoded = static_cast<int16_t>(rounded);
    return true;
}

/*
 * 将过载环角速度输出（rad/s）量化为 0.001 rad/s/LSB 的 int16。
 * 与 QuantizeInt16 不同：超出 int16 范围或非有限值时饱和到边界，而不是失败，
 * 避免瞬时大指令导致整帧遥测被丢弃。
 */
static int16_t QuantizeOverloadRateRadps(float value_radps)
{
    const float kOverloadRateLsbRadps = 0.001f;
    const float kOverloadRateMaxRadps = 32767.0f * kOverloadRateLsbRadps;
    if (value_radps != value_radps) {
        return 0;
    }

    float clamped = value_radps;
    if (clamped < -kOverloadRateMaxRadps) {
        clamped = -kOverloadRateMaxRadps;
    } else if (clamped > kOverloadRateMaxRadps) {
        clamped = kOverloadRateMaxRadps;
    }

    int16_t encoded = 0;
    if (!QuantizeInt16(clamped, 1000.0f, encoded)) {
        encoded = 0;
    }
    return encoded;
}

/**
 * @brief 将归一化机体控制量转换为四舍五入后的 uint8 线上值。
 */
static bool QuantizeBodyControlByte(float value, uint8_t& encoded)
{
    if (!(value >= -1.0f && value <= 1.0f)) {
        return false;
    }

    const float scaled = (value + 1.0f) * 127.5f;
    encoded = static_cast<uint8_t>(scaled + 0.5f);
    return true;
}

/** @copydoc HostProtocolCrc16 */
uint16_t HostProtocolCrc16(const uint8_t* data, uint16_t length)
{
    if (data == nullptr && length != 0U) {
        return 0U;
    }

    uint16_t crc = 0xFFFFU;
    for (uint16_t index = 0U; index < length; ++index) {
        crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[index]) << 8U);
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

/** @copydoc Class_HostControlFrameParser::Class_HostControlFrameParser */
Class_HostControlFrameParser::Class_HostControlFrameParser()
    : frame_(),
      frame_index_(0U),
      crc_error_count_(0U),
      format_error_count_(0U)
{
}

/** @copydoc Class_HostControlFrameParser::Feed */
bool Class_HostControlFrameParser::Feed(uint8_t byte, HostOverloadCommand& command)
{
    if (frame_index_ == 0U) {
        if (byte == kControlHeader0) {
            frame_[0] = byte;
            frame_index_ = 1U;
        }
        return false;
    }

    if (frame_index_ == 1U) {
        if (byte == kControlHeader1) {
            frame_[1] = byte;
            frame_index_ = 2U;
        } else if (byte != kControlHeader0) {
            frame_index_ = 0U;
        }
        return false;
    }

    frame_[frame_index_] = byte;
    ++frame_index_;
    if (frame_index_ < kHostControlFrameSize) {
        return false;
    }

    const bool valid = ValidateAndDecode(command);
    frame_index_ = (byte == kControlHeader0) ? 1U : 0U;
    if (frame_index_ == 1U) {
        frame_[0] = byte;
    }
    return valid;
}

/** @copydoc Class_HostControlFrameParser::Reset */
void Class_HostControlFrameParser::Reset()
{
    frame_index_ = 0U;
}

/** @copydoc Class_HostControlFrameParser::GetCrcErrorCount */
uint32_t Class_HostControlFrameParser::GetCrcErrorCount() const
{
    return crc_error_count_;
}

/** @copydoc Class_HostControlFrameParser::GetFormatErrorCount */
uint32_t Class_HostControlFrameParser::GetFormatErrorCount() const
{
    return format_error_count_;
}

/** @copydoc Class_HostControlFrameParser::ValidateAndDecode */
bool Class_HostControlFrameParser::ValidateAndDecode(HostOverloadCommand& command)
{
    if (frame_[12] != kFooter0 || frame_[13] != kFooter1) {
        ++format_error_count_;
        return false;
    }

    const uint16_t received_crc = static_cast<uint16_t>(frame_[10]) |
                                  static_cast<uint16_t>(static_cast<uint16_t>(frame_[11]) << 8U);
    const uint16_t calculated_crc = HostProtocolCrc16(&frame_[2], 8U);
    if (received_crc != calculated_crc) {
        ++crc_error_count_;
        return false;
    }

    command.is_launched = frame_[2];
    command.is_base_green_detected = frame_[3];
    command.yaw_overload_g = static_cast<float>(ReadInt16Le(&frame_[4])) / 100.0f;
    command.pitch_overload_g = static_cast<float>(ReadInt16Le(&frame_[6])) / 100.0f;
    command.reserved0 = frame_[8];
    command.reserved1 = frame_[9];
    command.valid = true;
    return true;
}

/** @copydoc HostProtocolEncodeImuFrame */
ResultCode HostProtocolEncodeImuFrame(
    const ImuData& imu,
    const BodyControlCommand& command,
    float overload_yaw_rate_radps,
    float overload_pitch_rate_radps,
    uint8_t* frame,
    uint16_t capacity)
{
    if (frame == nullptr || capacity < kHostImuFrameSize || !imu.valid) {
        return ResultCode::InvalidData;
    }

    int16_t encoded[6] = {};
    uint8_t encoded_pitch = 0x80U;
    uint8_t encoded_yaw = 0x80U;
    const float kRadiansPerSecondToTenthsDegreesPerSecond = 572.9577951f;
    // imu 已经是机体坐标系数据；协议层只按 X/Y/Z 顺序量化，不再换轴或取负。
    if (!QuantizeInt16(imu.specific_force_x_mps2, 100.0f, encoded[0]) ||
        !QuantizeInt16(imu.specific_force_y_mps2, 100.0f, encoded[1]) ||
        !QuantizeInt16(imu.specific_force_z_mps2, 100.0f, encoded[2]) ||
        !QuantizeInt16(imu.gyro_x_radps, kRadiansPerSecondToTenthsDegreesPerSecond, encoded[3]) ||
        !QuantizeInt16(imu.gyro_y_radps, kRadiansPerSecondToTenthsDegreesPerSecond, encoded[4]) ||
        !QuantizeInt16(imu.gyro_z_radps, kRadiansPerSecondToTenthsDegreesPerSecond, encoded[5])) {
        return ResultCode::InvalidData;
    }
    if (command.valid &&
        (!QuantizeBodyControlByte(command.pitch, encoded_pitch) ||
         !QuantizeBodyControlByte(command.yaw, encoded_yaw))) {
        return ResultCode::InvalidData;
    }

    frame[0] = kImuHeader0;
    frame[1] = kImuHeader1;
    for (uint16_t index = 0U; index < 6U; ++index) {
        WriteInt16Le(encoded[index], &frame[2U + index * 2U]);
    }
    frame[14] = encoded_pitch;
    frame[15] = encoded_yaw;
    WriteInt16Le(QuantizeOverloadRateRadps(overload_yaw_rate_radps), &frame[16]);
    WriteInt16Le(QuantizeOverloadRateRadps(overload_pitch_rate_radps), &frame[18]);
    WriteInt32Le(imu.data_ready_seq, &frame[20]);
    frame[24] = kImuTriggerFlagPlaceholder;

    const uint16_t crc = HostProtocolCrc16(&frame[2], 23U);
    frame[25] = static_cast<uint8_t>(crc & 0xFFU);
    frame[26] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
    frame[27] = kFooter0;
    frame[28] = kFooter1;
    return ResultCode::Ok;
}

ResultCode VofaProtocolEncodeJustFloatFrame(
    const ImuData& imu,
    uint8_t* frame,
    uint16_t capacity)
{
    if (frame == nullptr || capacity < kVofaJustFloatFrameSize || !imu.valid) {
        return ResultCode::InvalidData;
    }

    // imu 已经是机体坐标系数据；VOFA 只按 X/Y/Z 顺序发送，不再执行坐标变换。
    const float channels[kVofaJustFloatChannelCount] = {
        imu.specific_force_x_mps2,
        imu.specific_force_y_mps2,
        imu.specific_force_z_mps2,
        imu.gyro_x_radps,
        imu.gyro_y_radps,
        imu.gyro_z_radps
    };
    for (uint16_t index = 0U; index < kVofaJustFloatChannelCount; ++index) {
        if (!IsFiniteFloat(channels[index])) {
            return ResultCode::InvalidData;
        }
        WriteFloatLe(channels[index], &frame[index * 4U]);
    }

    frame[24] = kVofaJustFloatTail0;
    frame[25] = kVofaJustFloatTail1;
    frame[26] = kVofaJustFloatTail2;
    frame[27] = kVofaJustFloatTail3;
    return ResultCode::Ok;
}
