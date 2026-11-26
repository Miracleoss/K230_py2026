#include "Protocol/dart_light_protocol.h"

// CRC-8/MAXIM-DOW 查找表（多项式 x^8 + x^5 + x^4 + 1，反转多项式 0x8C，初值 0x00）
static const uint8_t kDartLightCrc8Table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35
};

uint8_t DartLightCrc8(const uint8_t* data, uint16_t length)
{
    if (data == nullptr || length == 0U) {
        return 0x00;
    }

    uint8_t crc = 0x00;
    for (uint16_t i = 0U; i < length; ++i) {
        crc = kDartLightCrc8Table[crc ^ data[i]];
    }
    return crc;
}

ResultCode DartLightEncodeQueryStatus(uint8_t* frame, uint16_t capacity, uint16_t& out_length)
{
    if (frame == nullptr || capacity < 4U) {
        out_length = 0U;
        return ResultCode::InvalidData;
    }

    frame[0] = kDartLightHeader;
    frame[1] = kDartLightCmdQueryStatus;
    frame[2] = 0x00; // 数据长度 0
    frame[3] = DartLightCrc8(frame, 3U);
    out_length = 4U;
    return ResultCode::Ok;
}

ResultCode DartLightEncodeSetTeam(DartLightTeam team, uint8_t* frame, uint16_t capacity, uint16_t& out_length)
{
    if (frame == nullptr || capacity < 5U) {
        out_length = 0U;
        return ResultCode::InvalidData;
    }

    frame[0] = kDartLightHeader;
    frame[1] = kDartLightCmdSetTeam;
    frame[2] = 0x01; // 数据长度 1
    frame[3] = (team == DartLightTeam::Blue) ? 0x01 : 0x00;
    frame[4] = DartLightCrc8(frame, 4U);
    out_length = 5U;
    return ResultCode::Ok;
}

ResultCode DartLightDecodeStatus(const DartLightFrame& frame, DartLightStatus& status)
{
    if (frame.cmd != kDartLightCmdQueryStatus || frame.len != 3U) {
        return ResultCode::InvalidData;
    }

    status.version = frame.data[0];
    status.temp_deg_c = static_cast<int8_t>(frame.data[1]);
    status.raw_state = frame.data[2];

    // 位解析:
    // Bit3: 0 红方, 1 蓝方
    // Bit2: 0 熄灭, 1 点亮
    // Bit1: 0 正常, 1 电压过低
    // Bit0: 0 正常, 1 异常
    status.team = ((status.raw_state & 0x08U) != 0U) ? DartLightTeam::Blue : DartLightTeam::Red;
    status.visible_led_on = ((status.raw_state & 0x04U) != 0U);
    status.supply_voltage_low = ((status.raw_state & 0x02U) != 0U);
    status.ir_led_fault = ((status.raw_state & 0x01U) != 0U);
    status.valid = true;

    return ResultCode::Ok;
}

ResultCode DartLightDecodeSetTeamAck(const DartLightFrame& frame, bool& success)
{
    if (frame.cmd != kDartLightCmdSetTeam || frame.len != 1U) {
        success = false;
        return ResultCode::InvalidData;
    }

    // 0x00: 设置成功; 0x01: 设置失败
    success = (frame.data[0] == 0x00);
    return ResultCode::Ok;
}

Class_DartLightFrameParser::Class_DartLightFrameParser()
    : step_(ParseStep::WaitHeader),
      raw_buffer_(),
      raw_index_(0U),
      expected_data_len_(0U),
      data_received_count_(0U),
      crc_error_count_(0U),
      format_error_count_(0U)
{
}

void Class_DartLightFrameParser::Reset()
{
    step_ = ParseStep::WaitHeader;
    raw_index_ = 0U;
    expected_data_len_ = 0U;
    data_received_count_ = 0U;
}

bool Class_DartLightFrameParser::Feed(uint8_t byte, DartLightFrame& frame)
{
    switch (step_) {
    case ParseStep::WaitHeader:
        if (byte == kDartLightHeader) {
            raw_buffer_[0] = byte;
            raw_index_ = 1U;
            step_ = ParseStep::WaitCmd;
        }
        return false;

    case ParseStep::WaitCmd:
        raw_buffer_[raw_index_++] = byte;
        step_ = ParseStep::WaitLen;
        return false;

    case ParseStep::WaitLen:
        if (byte > kDartLightMaxPayloadSize) {
            // 超出最大支持载荷
            ++format_error_count_;
            Reset();
            return false;
        }
        raw_buffer_[raw_index_++] = byte;
        expected_data_len_ = byte;
        data_received_count_ = 0U;
        if (expected_data_len_ == 0U) {
            step_ = ParseStep::WaitCrc;
        } else {
            step_ = ParseStep::WaitData;
        }
        return false;

    case ParseStep::WaitData:
        raw_buffer_[raw_index_++] = byte;
        ++data_received_count_;
        if (data_received_count_ >= expected_data_len_) {
            step_ = ParseStep::WaitCrc;
        }
        return false;

    case ParseStep::WaitCrc: {
        const uint8_t received_crc = byte;
        const uint8_t calculated_crc = DartLightCrc8(raw_buffer_, raw_index_);
        if (received_crc == calculated_crc) {
            // 校验通过，填充帧数据
            frame.cmd = raw_buffer_[1];
            frame.len = raw_buffer_[2];
            for (uint8_t i = 0U; i < frame.len; ++i) {
                frame.data[i] = raw_buffer_[3U + i];
            }
            frame.crc = received_crc;
            Reset();
            return true;
        }

        ++crc_error_count_;
        Reset();
        return false;
    }

    default:
        Reset();
        return false;
    }
}

uint32_t Class_DartLightFrameParser::GetCrcErrorCount() const
{
    return crc_error_count_;
}

uint32_t Class_DartLightFrameParser::GetFormatErrorCount() const
{
    return format_error_count_;
}

