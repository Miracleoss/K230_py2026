#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Common/control_command.h"
#include "Protocol/host_serial_protocol.h"

static bool Expect(bool condition, const char* message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        return false;
    }
    return true;
}

static bool ExpectNear(float actual, float expected, float tolerance, const char* message)
{
    return Expect(fabsf(actual - expected) <= tolerance, message);
}

int main()
{
    bool passed = true;

    const uint8_t crc_reference[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    passed &= Expect(
        HostProtocolCrc16(crc_reference, sizeof(crc_reference)) == 0x29B1U,
        "CRC-16/CCITT-FALSE reference vector must equal 0x29B1");

    const uint8_t valid_control_frame[kHostControlFrameSize] = {
        0xAAU, 0x55U,
        0xB5U, 0xFFU,  // roll = -75 -> -0.75 g
        0x96U, 0x00U,  // yaw = 150 -> 1.50 g
        0x01U, 0xFFU,  // pitch = -255 -> -2.55 g
        0x00U, 0x00U,
        0x3EU, 0x9DU,
        0x0DU, 0x0AU
    };

    Class_HostControlFrameParser parser;
    HostOverloadCommand command;
    uint32_t published_count = 0U;

    const uint8_t leading_noise[] = {0x00U, 0xAAU, 0x10U, 0xFFU};
    for (uint16_t index = 0U; index < sizeof(leading_noise); ++index) {
        if (parser.Feed(leading_noise[index], command)) {
            ++published_count;
        }
    }
    for (uint16_t index = 0U; index < 5U; ++index) {
        if (parser.Feed(valid_control_frame[index], command)) {
            ++published_count;
        }
    }
    passed &= Expect(published_count == 0U, "a fragmented frame must not publish early");
    for (uint16_t index = 5U; index < sizeof(valid_control_frame); ++index) {
        if (parser.Feed(valid_control_frame[index], command)) {
            ++published_count;
        }
    }
    passed &= Expect(published_count == 1U, "a complete valid frame must publish once");
    passed &= Expect(command.is_launched == 0xB5U && command.IsLaunched(), "byte 2 must decode as is_launched");
    passed &= Expect(command.is_base_green_detected == 0xFFU && command.IsBaseGreenDetected(), "byte 3 must decode as is_base_green_detected");
    passed &= ExpectNear(command.yaw_overload_g, 1.50f, 0.0001f, "yaw must decode as signed little-endian g");
    passed &= ExpectNear(command.pitch_overload_g, -2.55f, 0.0001f, "pitch must decode as signed little-endian g");
    passed &= Expect(command.reserved0 == 0U && command.reserved1 == 0U,
                     "reserved command bytes must be preserved without affecting validity");

    for (uint16_t repetition = 0U; repetition < 2U; ++repetition) {
        for (uint16_t index = 0U; index < sizeof(valid_control_frame); ++index) {
            if (parser.Feed(valid_control_frame[index], command)) {
                ++published_count;
            }
        }
    }
    passed &= Expect(published_count == 3U, "back-to-back valid frames must each publish");

    uint8_t invalid_frame[kHostControlFrameSize] = {};
    memcpy(invalid_frame, valid_control_frame, sizeof(invalid_frame));
    invalid_frame[10] ^= 0x01U;
    for (uint16_t index = 0U; index < sizeof(invalid_frame); ++index) {
        if (parser.Feed(invalid_frame[index], command)) {
            ++published_count;
        }
    }
    passed &= Expect(published_count == 3U, "a bad CRC must not publish a command");
    passed &= Expect(parser.GetCrcErrorCount() == 1U, "a bad CRC must increment its diagnostic counter");

    memcpy(invalid_frame, valid_control_frame, sizeof(invalid_frame));
    invalid_frame[13] = 0x00U;
    for (uint16_t index = 0U; index < sizeof(invalid_frame); ++index) {
        if (parser.Feed(invalid_frame[index], command)) {
            ++published_count;
        }
    }
    passed &= Expect(published_count == 3U, "a bad footer must not publish a command");
    passed &= Expect(parser.GetFormatErrorCount() >= 1U, "a bad footer must increment its diagnostic counter");

    const uint8_t full_range_control_frame[kHostControlFrameSize] = {
        0xAAU, 0x55U,
        0x00U, 0x80U,  // roll = INT16_MIN -> -327.68 g
        0xFFU, 0x7FU,  // yaw = INT16_MAX -> +327.67 g
        0x00U, 0x00U,
        0xABU, 0xCDU,
        0x62U, 0x7FU,
        0x0DU, 0x0AU
    };
    bool full_range_published = false;
    for (uint16_t index = 0U; index < sizeof(full_range_control_frame); ++index) {
        full_range_published |= parser.Feed(full_range_control_frame[index], command);
    }
    passed &= Expect(full_range_published, "the full int16 overload range must not be business-clamped");
    passed &= Expect(command.is_launched == 0x00U && !command.IsLaunched(), "byte 2 0x00 must decode as not launched");
    passed &= Expect(command.is_base_green_detected == 0x80U && command.IsBaseGreenDetected(), "byte 3 0x80 must decode as green detected");
    passed &= ExpectNear(command.yaw_overload_g, 327.67f, 0.001f, "INT16_MAX must decode to +327.67 g");
    passed &= Expect(command.reserved0 == 0xABU && command.reserved1 == 0xCDU,
                     "nonzero reserved bytes must not invalidate an otherwise valid frame");

    ImuData imu;
    // The protocol consumes already converted body-frame channels in X/Y/Z order.
    imu.specific_force_x_mps2 = 1.23f;
    imu.specific_force_y_mps2 = -4.56f;
    imu.specific_force_z_mps2 = 7.89f;
    imu.gyro_x_radps = 1.745329252f;   // +100 deg/s
    imu.gyro_y_radps = -3.490658504f;  // -200 deg/s
    imu.gyro_z_radps = 3.141592654f;   // +180 deg/s
    imu.data_ready_seq = 0x12345678U;
    imu.valid = true;

    BodyControlCommand body_command;
    body_command.pitch = -0.25f;
    body_command.yaw = 0.50f;
    body_command.valid = true;

    uint8_t imu_frame[kHostImuFrameSize] = {};
    passed &= Expect(
        HostProtocolEncodeImuFrame(imu, body_command, 0.5f, -1.5f, imu_frame, sizeof(imu_frame)) == ResultCode::Ok,
        "a valid IMU sample and body control command must encode");

    const uint8_t expected_imu_frame[kHostImuFrameSize] = {
        0x55U, 0xAAU,
        0x7BU, 0x00U,
        0x38U, 0xFEU,
        0x15U, 0x03U,
        0xE8U, 0x03U,
        0x30U, 0xF8U,
        0x08U, 0x07U,
        0x60U, 0xBFU,
        0xF4U, 0x01U,
        0x24U, 0xFAU,
        0x78U, 0x56U,
        0x34U, 0x12U,
        0x01U, 0x15U,
        0x00U, 0x0DU,
        0x0AU
    };
    passed &= Expect(memcmp(imu_frame, expected_imu_frame, sizeof(imu_frame)) == 0,
                     "IMU frame must encode P/Y commands as mapped bytes");

    passed &= Expect(
        HostProtocolEncodeImuFrame(imu, body_command, 0.5f, -1.5f, imu_frame, sizeof(imu_frame) - 1U) ==
            ResultCode::InvalidData,
        "a short output buffer must be rejected");
    body_command.pitch = -1.01f;
    passed &= Expect(
        HostProtocolEncodeImuFrame(imu, body_command, 0.5f, -1.5f, imu_frame, sizeof(imu_frame)) ==
            ResultCode::InvalidData,
        "an out-of-range P command must not encode");
    body_command.pitch = -0.25f;
    body_command.yaw = 1.01f;
    passed &= Expect(
        HostProtocolEncodeImuFrame(imu, body_command, 0.5f, -1.5f, imu_frame, sizeof(imu_frame)) ==
            ResultCode::InvalidData,
        "an out-of-range Y command must not encode");
    body_command.yaw = 0.50f;
    body_command.valid = false;
    passed &= Expect(
        HostProtocolEncodeImuFrame(imu, body_command, 0.5f, -1.5f, imu_frame, sizeof(imu_frame)) ==
            ResultCode::Ok && imu_frame[14] == 0x80U && imu_frame[15] == 0x80U,
        "an invalid body command must encode safe neutral P/Y bytes");

    imu.valid = false;
    passed &= Expect(
        HostProtocolEncodeImuFrame(imu, body_command, 0.5f, -1.5f, imu_frame, sizeof(imu_frame)) == ResultCode::InvalidData,
        "an invalid IMU sample must not encode");
    body_command.valid = true;
    imu.valid = true;
    imu.specific_force_x_mps2 = NAN;
    passed &= Expect(
        HostProtocolEncodeImuFrame(imu, body_command, 0.5f, -1.5f, imu_frame, sizeof(imu_frame)) == ResultCode::InvalidData,
        "NaN IMU data must not encode");
    imu.specific_force_x_mps2 = 400.0f;
    passed &= Expect(
        HostProtocolEncodeImuFrame(imu, body_command, 0.5f, -1.5f, imu_frame, sizeof(imu_frame)) == ResultCode::InvalidData,
        "out-of-range fixed-point IMU data must not wrap");

    ImuData vofa_imu;
    vofa_imu.specific_force_x_mps2 = 1.0f;
    vofa_imu.specific_force_y_mps2 = 2.0f;
    vofa_imu.specific_force_z_mps2 = 3.0f;
    vofa_imu.gyro_x_radps = 4.0f;
    vofa_imu.gyro_y_radps = 5.0f;
    vofa_imu.gyro_z_radps = 6.0f;
    vofa_imu.valid = true;

    uint8_t vofa_frame[kVofaJustFloatFrameSize] = {};
    passed &= Expect(
        VofaProtocolEncodeJustFloatFrame(vofa_imu, vofa_frame, sizeof(vofa_frame)) == ResultCode::Ok,
        "a valid IMU sample must encode as a JustFloat frame");
    const uint8_t expected_vofa_frame[kVofaJustFloatFrameSize] = {
        0x00U, 0x00U, 0x80U, 0x3FU,
        0x00U, 0x00U, 0x00U, 0x40U,
        0x00U, 0x00U, 0x40U, 0x40U,
        0x00U, 0x00U, 0x80U, 0x40U,
        0x00U, 0x00U, 0xA0U, 0x40U,
        0x00U, 0x00U, 0xC0U, 0x40U,
        0x00U, 0x00U, 0x80U, 0x7FU
    };
    passed &= Expect(memcmp(vofa_frame, expected_vofa_frame, sizeof(vofa_frame)) == 0,
                     "JustFloat must encode six little-endian floats and its sync tail");
    passed &= Expect(vofa_frame[0] != 0x55U || vofa_frame[1] != 0xAAU,
                     "JustFloat must not prepend the Host protocol header");
    passed &= Expect(
        VofaProtocolEncodeJustFloatFrame(vofa_imu, vofa_frame, kVofaJustFloatFrameSize - 1U) ==
            ResultCode::InvalidData,
        "a short JustFloat output buffer must be rejected");
    vofa_imu.valid = false;
    passed &= Expect(
        VofaProtocolEncodeJustFloatFrame(vofa_imu, vofa_frame, sizeof(vofa_frame)) ==
            ResultCode::InvalidData,
        "an invalid IMU sample must not encode as JustFloat");
    vofa_imu.valid = true;
    vofa_imu.gyro_z_radps = INFINITY;
    passed &= Expect(
        VofaProtocolEncodeJustFloatFrame(vofa_imu, vofa_frame, sizeof(vofa_frame)) ==
            ResultCode::InvalidData,
        "a non-finite IMU value must not encode as JustFloat");

    if (!passed) {
        return 1;
    }

    printf("Host serial protocol checks passed.\n");
    return 0;
}
